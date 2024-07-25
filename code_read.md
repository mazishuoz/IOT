代码结构：
main里面：
control_work.c和hello_word_main.c是原来代码<br />
test_camera.c是深度相机连接到esp32c3上通过uart串口通信的测试代码<br />
test_camera.c已实现从深度相机中获得25*25的深度图存储到缓冲区中<br />
newcontrol_work.c是测试有深度图的分层代码，这个也是理论上的，具体还要根据实际进行修改<br />
all.c是理论上的由深度相机获得图片，之后在esp32-s3上处理之后发送给矩阵电机<br />