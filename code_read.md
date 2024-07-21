代码结构：
main里面：
control_work.c和hello_word_main.c是原来代码<br />
test_camera.c是深度相机连接到esp32c3上通过uart串口通信的测试代码<br />
test_camera.c已实现从深度相机中获得25*25的深度图存储到队列中<br />
newcontrol_work.c是测试有深度图的分层代码<br />