1、选择开发板
![alt text](image-1.png)

2、选择摄像头

![alt text](image.png)

3、修改摄像头相关配置
屏蔽![alt text](image-2.png)

#if CONFIG_EXAMPLE_SCCB_I2C_INIT_BY_APP
    // ESP_RETURN_ON_ERROR(i2c_new_master_bus(&s_bus_config, &s_i2cbus_handle), TAG, "failed to initialize i2c bus");
    ESP_RETURN_ON_ERROR(i2c_master_get_bus_handle(0, &s_i2cbus_handle), TAG, "Failed to get I2C bus handle");
    ESP_LOGI(TAG, "successd to initialize i2c bus!");

![alt text](image-3.png)

修改![alt text](image-4.png)