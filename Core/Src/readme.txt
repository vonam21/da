19/7 
đổi uint8_t data_SIM5[40] = "AT+CMQTTTOPIC=0,5\r\n";
//uint8_t data_SIM5[40] = "AT+CMQTTSUB=0,5,1\r\n";
và 
uint8_t data_SIM10[40] = "AT+CMQTTTOPIC=0,7\r\n";

sửa lại để chỉ khi nhận mqtt thì mới gửi data, chứ cái cũ thì mỗi khi publish nó sẽ gửi data 1 lần nữa


code đang còn bị nếu server gửi tín hiệu ngay lúc mqtt publish thì miss data_SIM10