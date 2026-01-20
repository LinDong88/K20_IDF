#include "esp_check.h"
#include "esp_display_panel.hpp"
#include "esp_lib_utils.h"
#include "lvgl.h"
#include "lvgl_v8_port.h"
#include "lv_demos.h"

static const char *TAG = "DF";
#include <errno.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "example_video_common.h"

#define CONFIG_EXAMPLE_FORMAT_NON_ENCODE 1

#if CONFIG_EXAMPLE_FORMAT_MJPEG
#define ENCODE_DEV_PATH        ESP_VIDEO_JPEG_DEVICE_NAME
#define STORAGE_IMAGE_FORMAT   V4L2_PIX_FMT_JPEG
#elif CONFIG_EXAMPLE_FORMAT_H264
#define ENCODE_DEV_PATH            ESP_VIDEO_H264_DEVICE_NAME
#define STORAGE_IMAGE_FORMAT       V4L2_PIX_FMT_H264
#elif CONFIG_EXAMPLE_FORMAT_NON_ENCODE
#define STORAGE_IMAGE_FORMAT       V4L2_PIX_FMT_RGB565
#endif

#define VIDEO_BUFFER_COUNT         2
#define VIDEO_ENCODER_BUFFER_COUNT 1
#define SKIP_STARTUP_FRAME_COUNT   2

/**
 * @brief The framebuffer type
 */
typedef struct usb_msc_fb {
    uint8_t *buf;
    uint8_t buf_index;
    size_t buf_bytesused;
    size_t width;
    size_t height;
    struct timeval timestamp;
} usb_msc_fb_t;

typedef struct usb_msc_storage {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[VIDEO_BUFFER_COUNT];
#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    int m2m_fd;
    uint8_t *m2m_cap_buffer;
#endif
    usb_msc_fb_t um_fb;
} usb_msc_storage_t;

static esp_err_t init_capture_video(usb_msc_storage_t *umsc)
{
    int fd;

    ESP_LOGW(TAG, "EXAMPLE_CAM_DEV_PATH %s" , EXAMPLE_CAM_DEV_PATH);
    fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    assert(fd >= 0);

    umsc->cap_fd = fd;
    umsc->format = STORAGE_IMAGE_FORMAT;

    return ESP_OK;
}

#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
static esp_err_t set_codec_control(int fd, uint32_t ctrl_class, uint32_t id, int32_t value)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    controls.ctrl_class = ctrl_class;
    controls.count = 1;
    controls.controls = control;
    control[0].id = id;
    control[0].value = value;

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set control: %" PRIu32, id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t init_codec_video(usb_msc_storage_t *umsc)
{
    int fd;
    const char *devpath = ENCODE_DEV_PATH;

    fd = open(devpath, O_RDONLY);
    assert(fd >= 0);

#if CONFIG_EXAMPLE_FORMAT_MJPEG
    set_codec_control(fd, V4L2_CID_JPEG_CLASS, V4L2_CID_JPEG_COMPRESSION_QUALITY, CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY);
#elif CONFIG_EXAMPLE_FORMAT_H264
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, CONFIG_EXAMPLE_H264_I_PERIOD);
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_BITRATE, CONFIG_EXAMPLE_H264_BITRATE);
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, CONFIG_EXAMPLE_H264_MIN_QP);
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, CONFIG_EXAMPLE_H264_MAX_QP);
#endif

    umsc->m2m_fd = fd;

    return ESP_OK;
}
#endif

static esp_err_t example_video_start(usb_msc_storage_t *umsc)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct v4l2_format init_format;
    uint32_t capture_fmt = 0;
    uint32_t width, height;

    ESP_LOGD(TAG, "Video start");

    memset(&init_format, 0, sizeof(struct v4l2_format));
    init_format.type = type;
    if (ioctl(umsc->cap_fd, VIDIOC_G_FMT, &init_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        return ESP_FAIL;
    }
    width = init_format.fmt.pix.width;
    height = init_format.fmt.pix.height;

    if (umsc->format == V4L2_PIX_FMT_JPEG) {
        uint32_t fmt_index = 0;
        const uint32_t jpeg_input_formats[] = {
            V4L2_PIX_FMT_RGB565,
            V4L2_PIX_FMT_YUV422P,
            V4L2_PIX_FMT_RGB24,
            V4L2_PIX_FMT_GREY
        };
        uint32_t jpeg_input_formats_num = sizeof(jpeg_input_formats) / sizeof(jpeg_input_formats[0]);

        while (!capture_fmt) {
            struct v4l2_fmtdesc fmtdesc = {
                .index = fmt_index++,
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            };

            if (ioctl(umsc->cap_fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
                break;
            }

            for (int i = 0; i < jpeg_input_formats_num; i++) {
                if (jpeg_input_formats[i] == fmtdesc.pixelformat) {
                    capture_fmt = jpeg_input_formats[i];
                    break;
                }
            }
        }

        if (!capture_fmt) {
            ESP_LOGI(TAG, "The camera sensor output pixel format is not supported by JPEG encoder");
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else if (umsc->format == V4L2_PIX_FMT_H264) {
        capture_fmt = V4L2_PIX_FMT_YUV420;
    } else {
        capture_fmt = V4L2_PIX_FMT_RGB565;
    }

    /* Configure camera interface capture stream */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = VIDEO_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_REQBUFS, &req));

    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_ERROR_CHECK (ioctl(umsc->cap_fd, VIDIOC_QUERYBUF, &buf));

        umsc->cap_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                              MAP_SHARED, umsc->cap_fd, buf.m.offset);
        assert(umsc->cap_buffer[i]);

        ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_QBUF, &buf));
    }

#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    /* Configure codec output stream */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = VIDEO_ENCODER_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_REQBUFS, &req));

    /* Configure codec capture stream */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = umsc->format;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = 1;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_REQBUFS, &req));

    memset(&buf, 0, sizeof(buf));
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = 0;
    ESP_ERROR_CHECK (ioctl(umsc->m2m_fd, VIDIOC_QUERYBUF, &buf));

    umsc->m2m_cap_buffer = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                           MAP_SHARED, umsc->m2m_fd, buf.m.offset);
    assert(umsc->m2m_cap_buffer);

    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_QBUF, &buf));

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_STREAMON, &type));
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_STREAMON, &type));
#endif
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_STREAMON, &type));

    /* Skip the first few frames of the image to get a stable image. */
    for (int i = 0; i < SKIP_STARTUP_FRAME_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_DQBUF, &buf));
        ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_QBUF, &buf));
    }

    /* Init frame buffer's basic info. */
    umsc->um_fb.width = width;
    umsc->um_fb.height = height;

    return ESP_OK;
}

static usb_msc_fb_t *example_video_fb_get(usb_msc_storage_t *umsc)
{
    struct v4l2_buffer cap_buf;
#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    struct v4l2_buffer m2m_out_buf;
    struct v4l2_buffer m2m_cap_buf;
#endif
    int64_t us;

    ESP_LOGD(TAG, "Video get");

    memset(&cap_buf, 0, sizeof(cap_buf));
    cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_DQBUF, &cap_buf));
    
#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
    m2m_out_buf.index  = 0;
    m2m_out_buf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m2m_out_buf.memory = V4L2_MEMORY_USERPTR;
    m2m_out_buf.m.userptr = (unsigned long)umsc->cap_buffer[cap_buf.index];
    m2m_out_buf.length = cap_buf.bytesused;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_QBUF, &m2m_out_buf));

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf));

    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_QBUF, &cap_buf));
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_DQBUF, &m2m_out_buf));
    
    umsc->um_fb.buf = umsc->m2m_cap_buffer;
    umsc->um_fb.buf_bytesused = m2m_cap_buf.bytesused;
    umsc->um_fb.buf_index = m2m_cap_buf.index;
#else
    umsc->um_fb.buf = umsc->cap_buffer[cap_buf.index];
    umsc->um_fb.buf_bytesused = cap_buf.bytesused;
    umsc->um_fb.buf_index = cap_buf.index;
#endif
    us = esp_timer_get_time();
    umsc->um_fb.timestamp.tv_sec = us / 1000000UL;
    umsc->um_fb.timestamp.tv_usec = us % 1000000UL;

    return &umsc->um_fb;
}

static void example_video_fb_return(usb_msc_storage_t *umsc)
{
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.index  = umsc->um_fb.buf_index;
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    ESP_LOGD(TAG, "Video return");
#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    ESP_ERROR_CHECK(ioctl(umsc->m2m_fd, VIDIOC_QBUF, &buf));
#else
    ESP_ERROR_CHECK(ioctl(umsc->cap_fd, VIDIOC_QBUF, &buf));
#endif
}

using namespace esp_panel::drivers;
using namespace esp_panel::board;

void *frame_buffer = NULL;
size_t frame_buffer_size = 0;

static lv_img_dsc_t img_dsc;
static lv_obj_t *lv_img = NULL;

#define DISP_HRES 480
#define DISP_VRES 800

// pointCloudGrid: 0=空(白), 1=障碍(黑), 2=路径(红)
#define ROWS 20
#define COLS 32
static int pointCloudGrid[ROWS][COLS] = {
    {0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,2,2,2,2,2,2,2,2,2,2,2,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,2,2,2,2,2,2,2,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,2,2,2,2,2,2,2,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,2,2,2,2,2,2,2,0,0,0,0,1,1,1,1,1,0,0,0,1,0,0,0,0,1,1,0,1,1,0,0},
    {2,2,2,2,2,2,2,0,0,0,0,0,1,1,1,1,1,1,0,1,1,0,0,0,0,1,1,1,1,1,1,0},
    {2,2,2,2,2,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,0,0,0,0,1,1,1,1,1,1,1},
    {2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,1,1,1,1,1,1,1},
    {2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1},
    {2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1},
    {2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1}
};
static usb_msc_storage_t msc_ctrl;

#define UART_PORT UART_NUM_1
#define UART_TX_PIN GPIO_NUM_50
#define UART_RX_PIN GPIO_NUM_49
#define UART_BAUD_RATE 115200
#define UART_FRAME_DATA_LEN (ROWS * COLS)

static volatile bool s_grid_dirty = false;

static void uart_init_49_50(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void uart_rx_task(void *arg)
{
    (void)arg;

    enum {
        WAIT_HEAD = 0,
        READ_PAYLOAD,
        WAIT_TAIL,
    } state = WAIT_HEAD;

    uint8_t payload[UART_FRAME_DATA_LEN];
    size_t payload_idx = 0;

    while (1) {
        uint8_t b = 0;
        int n = uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }

        switch (state) {
        case WAIT_HEAD:
            if (b == 0xF5) {
                payload_idx = 0;
                state = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            payload[payload_idx++] = b;
            if (payload_idx >= UART_FRAME_DATA_LEN) {
                state = WAIT_TAIL;
            }
            break;

        case WAIT_TAIL:
            if (b == 0xAF) {
                for (int r = 0; r < ROWS; r++) {
                    for (int c = 0; c < COLS; c++) {
                        pointCloudGrid[r][c] = payload[r * COLS + c];
                    }
                }
                s_grid_dirty = true;
            }
            state = WAIT_HEAD;
            break;

        default:
            state = WAIT_HEAD;
            break;
        }
    }
}
static void draw_point_cloud_grid(void)
{
    // 蓝色线宽（像素），按实际视觉可调整
    const int line_w = 3;
    lv_obj_t *scr = lv_scr_act();

    // 旋转90°后：显示维度为 (ROWS x COLS)
    // 为了尽可能填满屏幕，这里允许非正方形格子：分别计算 cell_w / cell_h
    int cell_w = DISP_HRES / ROWS;
    int cell_h = DISP_VRES / COLS;
    if (cell_w < 1) {
        cell_w = 1;
    }
    if (cell_h < 1) {
        cell_h = 1;
    }

    const int grid_w = cell_w * ROWS;
    const int grid_h = cell_h * COLS;

    static lv_obj_t *canvas = NULL;
    static uint8_t *canvas_buf = NULL;

    if (canvas == NULL) {
        canvas = lv_canvas_create(scr);
        lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(canvas, grid_w, grid_h);
        lv_obj_center(canvas);

        size_t buf_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR(grid_w, grid_h);
        canvas_buf = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (canvas_buf == NULL) {
            // fallback to default heap
            canvas_buf = (uint8_t *)malloc(buf_size);
        }
        if (canvas_buf == NULL) {
            return;
        }
        lv_canvas_set_buffer(canvas, canvas_buf, grid_w, grid_h, LV_IMG_CF_TRUE_COLOR);
    } else {
        lv_obj_set_size(canvas, grid_w, grid_h);
        lv_obj_center(canvas);
    }

    // 先清屏为黑色
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.border_width = 0;
    rect_dsc.radius = 0;
    rect_dsc.bg_opa = LV_OPA_COVER;

    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int v = pointCloudGrid[r][c];
            if (v == 0) {
                rect_dsc.bg_color = lv_color_black();
            } else if (v == 1) {
                rect_dsc.bg_color = lv_color_white();
            } else {
                // rect_dsc.bg_color = lv_color_make(0xFF, 0x00, 0x00);
            }

            // 旋转90°顺时针：src(r,c) -> dst(x,y)
            // dst_col = ROWS - 1 - r
            // dst_row = c
            const int dst_col = ROWS - 1 - r;
            const int dst_row = c;

            const int x1 = dst_col * cell_w;
            const int y1 = dst_row * cell_h;
            lv_canvas_draw_rect(canvas, x1, y1, cell_w, cell_h, &rect_dsc);
        }
    }


    // --- 在路径上绘制更顺滑的中心线和边界线 ---
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color_make(0x00, 0x00, 0xFF);
    line_dsc.width = line_w;
    line_dsc.round_start = 1;
    line_dsc.round_end = 1;

    // 收集每一行路径段的中心/左右点，然后做简单平滑（3点移动平均）再连线
    lv_point_t center_pts[ROWS];
    lv_point_t left_pts[ROWS];
    lv_point_t right_pts[ROWS];
    uint8_t valid[ROWS] = {0};

    for (int r = 0; r < ROWS; r++) {
        int min_c = -1, max_c = -1;
        for (int c = 0; c < COLS; c++) {
            if (pointCloudGrid[r][c] == 2) {
                if (min_c < 0) min_c = c;
                max_c = c;
            }
        }
        if (min_c < 0) {
            continue;
        }

        const int dst_col_base = ROWS  - r;
        const float center_c = (min_c + max_c) * 0.5f;

        center_pts[r].x = (lv_coord_t)(dst_col_base * cell_w );
        center_pts[r].y = (lv_coord_t)(center_c * cell_h );

        // 这里的"左右边界"是在当前坐标系下对路径段上下边界连线，确保连续
        left_pts[r].x = (lv_coord_t)(dst_col_base * cell_w );
        left_pts[r].y = (lv_coord_t)(min_c * cell_h);

        right_pts[r].x = (lv_coord_t)(dst_col_base * cell_w );
        right_pts[r].y = (lv_coord_t)(max_c * cell_h);

        valid[r] = 1;
    }

    auto smooth3 = [](lv_point_t *pts, const uint8_t *v, int n) {
        for (int i = 1; i < n - 1; i++) {
            if (!v[i - 1] || !v[i] || !v[i + 1]) continue;
            pts[i].x = (lv_coord_t)((pts[i - 1].x + pts[i].x + pts[i + 1].x) / 3);
            pts[i].y = (lv_coord_t)((pts[i - 1].y + pts[i].y + pts[i + 1].y) / 3);
        }
    };

    smooth3(center_pts, valid, ROWS);
    smooth3(left_pts, valid, ROWS);
    smooth3(right_pts, valid, ROWS);

    // 绘制由四个点构成的多边形（连接相邻行的路径边界）
    for (int r = 0; r < ROWS - 1; r++) {
        if (valid[r] && valid[r + 1]) {
            // 定义四个点构成的多边形
            lv_point_t points[] = {
                left_pts[r],      // 当前行左边点
                right_pts[r],     // 当前行右边点
                right_pts[r + 1], // 下一行右边点
                left_pts[r + 1]   // 下一行左边点
            };

            // 设置红色填充
            rect_dsc.bg_color = lv_color_make(0xFF, 0x00, 0x00);

            // 绘制红色多边形
            lv_canvas_draw_polygon(canvas, points, 4, &rect_dsc);
        }
    }
    // 绘制最后一行到画布底部的多边形
    if (valid[ROWS - 1]) {  // 确保最后一行有效
        lv_point_t bottom_poly[] = {
            left_pts[ROWS - 1],           // 最后一行左边点
            right_pts[ROWS - 1],          // 最后一行右边点
            {0, right_pts[ROWS - 1].y }, // 
            {0, left_pts[ROWS - 1].y}   // 
        };
        
        // 设置红色填充
        rect_dsc.bg_color = lv_color_make(0xFF, 0x00, 0x00);
        // 绘制红色多边形到边框
        lv_canvas_draw_polygon(canvas, bottom_poly, 4, &rect_dsc);
    }

    lv_point_t last_center = {-1, -1};
    lv_point_t last_left = {-1, -1};
    lv_point_t last_right = {-1, -1};

    for (int r = 0; r < ROWS; r++) {
        if (!valid[r]) continue;

        lv_point_t cur_center = center_pts[r];
        lv_point_t cur_left = left_pts[r];
        lv_point_t cur_right = right_pts[r];

        if (last_center.x != -1) {
            lv_point_t pts_c[2] = {last_center, cur_center};
            lv_point_t pts_l[2] = {last_left, cur_left};
            lv_point_t pts_r[2] = {last_right, cur_right};
            lv_canvas_draw_line(canvas, pts_c, 2, &line_dsc);
            lv_canvas_draw_line(canvas, pts_l, 2, &line_dsc);
            lv_canvas_draw_line(canvas, pts_r, 2, &line_dsc);
        }

        last_center = cur_center;
        last_left = cur_left;
        last_right = cur_right;
    }
}

extern "C" void app_main(void)
{
    frame_buffer_size = DISP_HRES * DISP_VRES * 2;
    ESP_LOGD(TAG, "frame_buffer_size: %zu", frame_buffer_size);

    // 分配摄像头图像帧缓冲区内存
    frame_buffer = heap_caps_malloc(frame_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!frame_buffer) {
        ESP_LOGE(TAG, "Not enough memory for frame buffer");
        return;
    }
    
    Board *board = new Board();
    assert(board);

    ESP_LOGI(TAG, "Initializing board");
    ESP_UTILS_CHECK_FALSE_EXIT(board->init(), "Board init failed");
    ESP_UTILS_CHECK_FALSE_EXIT(board->begin(), "Board begin failed");

    auto backlight = board->getBacklight();
    if (backlight) {
        backlight->setBrightness(20); //背光
    }


    ESP_LOGI(TAG, "Initializing LVGL");
    ESP_UTILS_CHECK_FALSE_EXIT(lvgl_port_init(board->getLCD(), board->getTouch()), "LVGL init failed");

    uart_init_49_50();
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);

    if (lvgl_port_lock(-1)) {
        draw_point_cloud_grid();
        lvgl_port_unlock();
    }

    while (1) {
        if (s_grid_dirty) {
            s_grid_dirty = false;
            if (lvgl_port_lock(-1)) {
                draw_point_cloud_grid();
                lvgl_port_unlock();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

#if 0
    // 初始化图像描述符
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = DISP_HRES;
    img_dsc.header.h = DISP_VRES;
    img_dsc.data_size = DISP_HRES * DISP_VRES * 2;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data = (uint8_t *)frame_buffer;

    // 创建LVGL图像对象
    lv_img = lv_img_create(lv_scr_act());
    lv_img_set_src(lv_img, &img_dsc);
    lv_obj_set_size(lv_img, DISP_HRES, DISP_VRES);
    lv_obj_center(lv_img);

    ESP_LOGI(TAG, "Initializing video system");
    ESP_ERROR_CHECK(example_video_init());
    ESP_ERROR_CHECK(init_capture_video(&msc_ctrl));
#if !CONFIG_EXAMPLE_FORMAT_NON_ENCODE
    ESP_ERROR_CHECK(init_codec_video(&msc_ctrl));
#endif

    ESP_ERROR_CHECK(example_video_start(&msc_ctrl));
    
    // 预计算裁剪参数
    const int cam_width = msc_ctrl.um_fb.width;   // 800
    const int cam_height = msc_ctrl.um_fb.height; // 800
    const int disp_width = DISP_HRES;             // 380
    const int disp_height = DISP_VRES;            // 700
    
    // 计算中心裁剪的起始位置
    const int crop_x = (cam_width - disp_width) / 2;  // 160
    const int crop_y = (cam_height - disp_height) / 2; // 0
    
    // 预计算内存地址偏移
    const int src_start_offset = crop_y * cam_width + crop_x;
    
    while (1) {
        // 获取摄像头帧数据
        usb_msc_fb_t *fb = example_video_fb_get(&msc_ctrl);
        
        if (fb && fb->buf) {
            uint16_t *src = (uint16_t*)fb->buf;
            uint16_t *dst = (uint16_t*)frame_buffer;
            
            const int src_width = cam_width;    // 800
            const int src_height = cam_height;  // 800
            const int dst_width = DISP_HRES;    // 400
            const int dst_height = DISP_VRES;   // 400
            
            // 预计算缩放因子，使用定点运算避免浮点运算
            const int scale_x_fixed = (src_width << 16) / dst_width;  // x方向缩放因子(定点)
            const int scale_y_fixed = (src_height << 16) / dst_height; // y方向缩放因子(定点)
            
            for (int y = 0; y < dst_height; y++) {
                // 计算当前行对应的源图像y坐标
                int src_y = (y * scale_y_fixed) >> 16;
                uint16_t *src_row = src + src_y * src_width;  // 指向源图像对应行
                uint16_t *dst_row = dst + y * dst_width;      // 指向目标图像对应行
                
                for (int x = 0; x < dst_width; x++) {
                    // 计算当前像素对应的源图像x坐标
                    int src_x = (x * scale_x_fixed) >> 16;
                    dst_row[x] = src_row[src_x];
                }
            }
            
            // 更新LVGL图像显示
            lv_img_set_src(lv_img, &img_dsc);
        }
        
        // 归还帧缓冲区给摄像头驱动
        example_video_fb_return(&msc_ctrl);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
#endif
}