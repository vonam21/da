const express = require('express');
const bodyParser = require('body-parser');
const mqtt = require('mqtt');


const app = express();
app.use(bodyParser.urlencoded({ extended: true }));
var status_mode = 0;
var status_led = 0;
var status_quat = 0;
var status_maybom = 0;
var check_led = 0;
var check_quat = 0;
var check_bom = 0;
var status_ADCquangtro = 3000;
var status_ADCdoamdat = 3000;
var nguong_nhiet_do = 30;
var tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
    ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
    ' ' + nguong_nhiet_do;

var message_ser = 'none none none none none none none none'
var values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
app.set('view engine', 'ejs');
app.set('views', './views');
const client = mqtt.connect('mqtt://test.mosquitto.org');
const server = app.listen(3000, () => {
    console.log('Server listening on port 3000');
});
const io = require('socket.io')(server);


client.on('connect', () => {
    console.log('Connected to MQTT broker');

    // Subscribe vào chủ đề 'vonam' khi kết nối thành công
    client.subscribe('vonam', (err) => {
        if (!err) {
            console.log('Subscribed to vonam');
        }
    });
    // Gửi dữ liệu mới tới client qua Socket.IO
});
var cho_phep_gui_mqtt_led = 1;
var cho_phep_gui_mqtt_quat = 1;
var cho_phep_gui_mqtt_bom = 1;
var cho_phep_gui_mqtt_mode = 1;

var nguong_adc_quang_tro = 0;
var nguong_adc_do_am_dat = 0;
var nguong_nhiet_do1 = 0;
client.on('message', (topic, message) => {
    message_ser = message.toString();
    values = message_ser.split(' ');
    // values sẽ là một mảng chứa các giá trị riêng biệt
    console.log(`Received message on topic ${topic}: ${message_ser}`);
    cho_phep_gui_mqtt_mode = 0;
    cho_phep_gui_mqtt_led = 0;
    cho_phep_gui_mqtt_quat = 0;
    cho_phep_gui_mqtt_bom = 0;
    io.emit('mqttMessage', {
        nhietdo: values[0],
        doam: values[1],
        adc: values[2],
        adc_doam: values[3],
        nguong_adc_quang_tro: values[4],
        nguong_adc_do_am_dat: values[5],
        nguong_nhiet_do1: values[6],
        status_led: values[7],
        status_quat: values[8],
        status_bom: values[9],
        mode_hoat_dong: values[10],
        check_led: values[11],
        check_quat: values[12],
        check_bom: values[13],
    });
    console.log('%d = %d', values[9], values[10]);
});

// Sử dụng bodyParser để đọc dữ liệu từ yêu cầu POST
app.use(bodyParser.json());

var a = 1;

app.get('/', (req, res) => {
    console.log('This is Nam');
    const nhietdo = values[0];
    const doam = values[1];
    const adc = values[2];
    const adc_doam = values[3];
    const mode_hoat_dong = values[4];
    const status_led = values[5];
    const status_quat = values[6];
    const status_bom = values[7];
    const check_led1 = values[8];
    const check_quat1 = values[9];
    const check_bom1 = values[10];
    if (a === 1) {
        res.render('index.ejs')
        console.log('index')
    } else {
        res.render('index3.ejs', { nhietdo, doam, adc, adc_doam })
        console.log('index3')
    }
});

// Route để xử lý yêu cầu POST từ trình duyệt
app.post('/updateStatus', (req, res) => {
    status_led = req.body.status;
    console.log('Received status led from browser:', status_led);
    // if (cho_phep_gui_mqtt_led === 0) {
    //     cho_phep_gui_mqtt_led = 1;
    // } else {
    cho_phep_gui_mqtt_led = 1;
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    //}

    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
    res.send('OK');
});
app.post('/updateMode', (req, res) => {
    status_mode = req.body.status;
    console.log('Received Mode  from browser:', status_mode);
    // if (cho_phep_gui_mqtt_mode === 0) {
    //   cho_phep_gui_mqtt_mode = 1;
    // } else {
    cho_phep_gui_mqtt_mode = 1;
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    //}

    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
    res.send('OK');
    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
});

app.post('/updateStatus1', (req, res) => {
    status_quat = req.body.status;
    console.log('Received status quat from browser:', status_quat);
    // if (cho_phep_gui_mqtt_quat === 0) {
    //     cho_phep_gui_mqtt_quat = 1;
    // } else {
    cho_phep_gui_mqtt_quat = 1;
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    // }

    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
    res.send('OK');
});

app.post('/updateMayBom', (req, res) => {
    status_maybom = req.body.status;
    console.log('Received status from browser máy bơm:', status_maybom);
    //   if (cho_phep_gui_mqtt_bom === 0) {
    //     cho_phep_gui_mqtt_bom = 1;
    //   } else {
    cho_phep_gui_mqtt_bom = 1;
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    //}

    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
    res.send('OK');
    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
});

app.post('/updateADCquangtro', (req, res) => {
    status_ADCquangtro = req.body.ADC_quang_tro;
    console.log(
        'Received status from browser adc quang trở:', status_ADCquangtro);
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    res.send('OK');
    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
});

app.post('/updatenhietdo', (req, res) => {
    nguong_nhiet_do = req.body.nhietdo;
    console.log('Received status from browser nhiet do:', nguong_nhiet_do);
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    res.send('OK');
    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
});

app.post('/updateADCdoamdat', (req, res) => {
    status_ADCdoamdat = req.body.ADC_do_am;
    console.log('Received status from browser adc độ ẩm đất:', status_ADCdoamdat);
    tong_chuoi_gui_mqtt = status_led + ' ' + status_quat + ' ' + status_maybom +
        ' ' + status_mode + ' ' + status_ADCquangtro + ' ' + status_ADCdoamdat +
        ' ' + nguong_nhiet_do;
    client.publish('nhietdo', tong_chuoi_gui_mqtt);
    res.send('OK');
    // Thực hiện các thao tác với status ở đây
    // Ví dụ: Gửi status tới MQTT broker
});
// Route để lấy giá trị mới từ server

app.post('/login', (req, res) => {
    const { username, password } = req.body;
    console.log(username);
    console.log(password);
    // Kiểm tra thông tin đăng nhập ở đây (ví dụ: kiểm tra username và password có
    // đúng không)
    if (username === 'admin' && password === 'admin123') {
        a = 0;
    } else {
        a = 1;
    }
    setTimeout(() => {
        res.redirect('/');
    }, 2000);
});