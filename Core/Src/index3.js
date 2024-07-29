// < !DOCTYPE html >

//     <
//     html >

//     <
//     head >
//     <
//     script src = "https://code.jquery.com/jquery-3.6.4.min.js" > < /script> <
// script src = "https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js" > < /script> < /
// head >

//     <
//     body >
//     <
//     h1 style = "font-size:300%;text-align:center" > DATA FROM STM32 < /h1> <
// p id = "nhietdo"
// style = "font-size:200%;text-align:center" > Nhiệt độ:
//     <%= nhietdo %>
// 0 C < /p> <
// p id = "doam"
// style = "font-size:200%;text-align:center" > Độ ẩm:
//     <%= doam%> % < /p> <
// p id = "adc"
// style = "font-size:200%;text-align:center" > ADC quang trở:
//     <%= adc%> <
//     /p> <
// p id = "adc_doam"
// style = "font-size:200%;text-align:center" > ADC Độ ẩm đất:
//     <%= adc_doam%> <
//     /p>

// <
// p style = "font-size:200%" > Mode hoạt động < /p> <
// input type = "radio"
// name = "onOff12"
// id = "on"
// value = "on"
// onchange = "updateMode('Tu Dong')" > Tự Động <
//     input type = "radio"
// name = "onOff12"
// id = "off"
// value = "off"
// onchange = "updateMode('Bang Tay')" > Bằng Tay <
//     p id = "mode_hoat_dong" > < /p>






// <
// p style = "font-size:200%" > LED < /p> <
// input type = "radio"
// name = "onOff"
// id = "on_led"
// value = "on"
// onchange = "updateStatus('On')" > On <
//     input type = "radio"
// name = "onOff"
// id = "off_led"
// value = "off"
// onchange = "updateStatus('Off')" > Off <
//     p id = "nhietdo1" > < /p>


// <
// p style = "font-size:200%" > Quat < /p> <
// input type = "radio"
// name = "onOff1"
// id = "on_quat"
// value = "on"
// onchange = "updateStatus1('On')" > On <
//     input type = "radio"
// name = "onOff1"
// id = "off_quat"
// value = "off"
// onchange = "updateStatus1('Off')" > Off <
//     p id = "quat1" > < /p>

// <
// p style = "font-size:200%" > May bom < /p> <
// input type = "radio"
// name = "onOff2"
// id = "on_may_bom"
// onchange = "updateMayBom('On')" > On <
//     input type = "radio"
// name = "onOff2"
// id = "off_may_bom"
// onchange = "updateMayBom('Off')" > Off <
//     p id = "may_bom" > < /p>

// <
// p style = "font-size:200%" > Time: < /p> <
// p id = "hvn"
// style = "font-size:150%" > đay là hiển thị thời gian < /p> <
// p id = "hvn2"
// style = "font-size:150%" > đay là hiển thị thời gian < /p>

// <
// form onsubmit = "updateSubmit(); return false;" >
//     Nhập giá trị ngưỡng ADC cho quang trở:
//     <
//     input type = "number"
// id = "ADCquangtro"
// placeholder = "nhập giá trị từ 0 - 4095" >
//     <
//     input type = "submit"
// value = "Submit" >
//     <
//     /form> <
// p id = "quang" > < /p>

// <
// form onsubmit = "updateSubmit2(); return false;" >
//     Nhập giá trị ngưỡng ADC cho độ ẩm đất:
//     <
//     input type = "number"
// id = "ADC do am dat"
// placeholder = "nhập giá trị từ 0 - 4095" >
//     <
//     input type = "submit"
// value = "Submit" >
//     <
//     /form> <
// p id = "dat" > < /p>

// <
// form onsubmit = "updateSubmit3(); return false;" >
//     Nhập giá trị ngưỡng cho nhiệt độ:
//     <
//     input type = "number"
// id = "nguong nhiet do"
// placeholder = "nhập giá trị từ 0 - 50" >
//     <
//     input type = "submit"
// value = "Submit" >
//     <
//     /form> <
// p id = "nguong_nhiet_do" > < /p>











// <
// script >
//     var mode_hoat_dong_tu_dong = 0;
// var mode_hoat_dong_bang_tay = 1;
// var status_cho_phep_dao = 0;

// function updateMode(status) {
//     // var statusElement = document.getElementById('mode_hoat_dong');
//     // statusElement.innerText = 'Mode hoạt động :' + status;



//     // Cập nhật trạng thái của ô radio dựa trên mode
//     var automaticRadioButton = document.getElementById('on');
//     var manualRadioButton = document.getElementById('off');

//     if (status === 'Tu Dong') {
//         status = 1;
//         automaticRadioButton.checked = true;
//         mode_hoat_dong_tu_dong = 1;
//         mode_hoat_dong_bang_tay = 0;
//     } else if (status === 'Bang Tay') {
//         status = 0;
//         manualRadioButton.checked = true;
//         mode_hoat_dong_tu_dong = 0;
//         mode_hoat_dong_bang_tay = 1;
//     }



//     // Gửi giá trị status lên server

//     fetch('/updateMode', {
//         method: 'POST',
//         headers: {
//             'Content-Type': 'application/json',
//         },
//         body: JSON.stringify({
//             status
//         }),
//     });
// }



// function updateStatus(status) {
//     if (mode_hoat_dong_bang_tay == '1' || status_cho_phep_dao == 1) {
//         status_cho_phep_dao = 0;

//         // var statusElement = document.getElementById('nhietdo1');
//         // statusElement.innerText ='Trạng thái LED :' + status;


//         var auto_click_on_cho_led = document.getElementById('on_led');
//         var auto_click_off_cho_led = document.getElementById('off_led');
//         if (status == 'On') {
//             status = 1;
//             auto_click_on_cho_led.checked = true;
//         }
//         if (status == 'Off') {
//             status = 0;
//             auto_click_off_cho_led.checked = true;
//         }
//         // Gửi giá trị status lên server

//         fetch('/updateStatus', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 status
//             }),
//         });
//     }

// }


// function updateStatus1(status) {
//     if (mode_hoat_dong_bang_tay == '1') {
//         // var statusElement = document.getElementById('quat1');
//         // statusElement.innerText ='Trạng thái Quat :' + status;

//         var auto_click_on_cho_quat = document.getElementById('on_quat');
//         var auto_click_off_cho_quat = document.getElementById('off_quat');
//         if (status == 'On') {
//             status = 1;
//             auto_click_on_cho_quat.checked = true;
//         }
//         if (status == 'Off') {
//             status = 0;
//             auto_click_off_cho_quat.checked = true;
//         }
//         // Gửi giá trị status lên server

//         fetch('/updateStatus1', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 status
//             }),
//         });
//     }
// }

// function updateMayBom(status) {
//     if (mode_hoat_dong_bang_tay == '1') {
//         // var statusElement = document.getElementById('may_bom');
//         // statusElement.innerText ='Trạng thái máy bơm :' + status;


//         var auto_click_on_cho_may_bom = document.getElementById('on_may_bom');
//         var auto_click_off_cho_may_bom = document.getElementById('off_may_bom');
//         if (status == 'On') {
//             status = 1;
//             auto_click_on_cho_may_bom.checked = true;
//         }
//         if (status == 'Off') {
//             status = 0;
//             auto_click_off_cho_may_bom.checked = true;
//         }


//         // Gửi giá trị status lên server
//         fetch('/updateMayBom', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 status
//             }),
//         });
//     }
// }

// function updateTime() {

//     var today = new Date();
//     var date = today.getDate() + '-' + (today.getMonth() + 1) + '-' + today.getFullYear();
//     var time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
//     document.getElementById("hvn").innerHTML = date;
//     document.getElementById("hvn2").innerHTML = time;
// }
// setInterval(updateTime, 1000);

// function updateSubmit() {
//     if (mode_hoat_dong_bang_tay == '1') {
//         var ADC_quang_tro = document.getElementById("ADCquangtro").value;
//         // document.getElementById('quang').innerText = 'Giá trị ngưỡng hiện tại của ADC quang trở:' + ADC_quang_tro
//         fetch('/updateADCquangtro', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 ADC_quang_tro
//             }),
//         });
//     }
// }

// function updateSubmit2() {
//     if (mode_hoat_dong_bang_tay == '1') {
//         var ADC_do_am = document.getElementById("ADC do am dat").value;
//         // document.getElementById('dat').innerText = 'Giá trị ngưỡng hiện tại của ADC do am dat:' + ADC_do_am
//         fetch('/updateADCdoamdat', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 ADC_do_am
//             }),
//         });
//     }
// }

// function updateSubmit3() {
//     if (mode_hoat_dong_bang_tay == '1') {
//         var nhietdo = document.getElementById("nguong nhiet do").value;
//         // document.getElementById('nguong_nhiet_do').innerText = 'Giá trị ngưỡng hiện tại của nhiệt độ:' + nhietdo
//         fetch('/updatenhietdo', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({
//                 nhietdo
//             }),
//         });
//     }
// }

// const socket = io();

// // Bắt sự kiện khi nhận được MQTT message qua WebSocket
// socket.on('mqttMessage', (data) => {
//     // Cập nhật dữ liệu trên trang khi có tin nhắn mới
//     console.log('Received MQTT message:', data);
//     document.getElementById('nhietdo').innerText = `Nhiệt độ: ${data.nhietdo} 0C`;
//     document.getElementById('doam').innerText = `Độ ẩm: ${data.doam}%`;
//     document.getElementById('adc').innerText = `ADC quang trở: ${data.adc}`;
//     document.getElementById('adc_doam').innerText = `ADC Độ ẩm đất: ${data.adc_doam}`;
//     if (data.mode_hoat_dong == '0') {
//         status = 'Bang Tay';
//         var statusElement = document.getElementById('mode_hoat_dong');
//         statusElement.innerText = 'Mode hoạt động :Bang tay';
//         var automaticRadioButton = document.getElementById('on');
//         var manualRadioButton = document.getElementById('off');

//         if (status === 'Tu Dong') {
//             status = 1;
//             automaticRadioButton.checked = true;
//             mode_hoat_dong_tu_dong = 1;
//             mode_hoat_dong_bang_tay = 0;
//         } else if (status === 'Bang Tay') {
//             status = 0;
//             manualRadioButton.checked = true;
//             mode_hoat_dong_tu_dong = 0;
//             mode_hoat_dong_bang_tay = 1;
//         }
//         // updateMode('Bang tay')
//     }
//     // if (data.status_led == '1') {
//     //     updateStatus('On')
//     // }
//     // if (data.status_led == '0') {
//     //     updateStatus('Off')
//     // }
//     // if (data.status_quat == '1') {
//     //     updateStatus1('On')
//     // }
//     // if (data.status_quat == '0') {
//     //     updateStatus1('Off')
//     // }
//     // if (data.status_bom == '1') {
//     //     updateMayBom('On')
//     // }
//     // if (data.status_bom == '0') {
//     //     updateMayBom('Off')
//     // }

//     if (data.mode_hoat_dong == '1') {
//         var statusElement = document.getElementById('mode_hoat_dong');
//         statusElement.innerText = 'Mode hoạt động :Tu dong';
//         var automaticRadioButton = document.getElementById('on');
//         var manualRadioButton = document.getElementById('off');
//         status = 'Tu Dong';
//         if (status === 'Tu Dong') {
//             status = 1;
//             automaticRadioButton.checked = true;
//             mode_hoat_dong_tu_dong = 1;
//             mode_hoat_dong_bang_tay = 0;
//         } else if (status === 'Bang Tay') {
//             status = 0;
//             manualRadioButton.checked = true;
//             mode_hoat_dong_tu_dong = 0;
//             mode_hoat_dong_bang_tay = 1;
//         }
//         // updateMode('Tu Dong')
//     }
//     var auto_click_on_cho_led = document.getElementById('on_led');
//     var auto_click_off_cho_led = document.getElementById('off_led');
//     var auto_click_on_cho_quat = document.getElementById('on_quat');
//     var auto_click_off_cho_quat = document.getElementById('off_quat');
//     var auto_click_on_cho_may_bom = document.getElementById('on_may_bom');
//     var auto_click_off_cho_may_bom = document.getElementById('off_may_bom');
//     if (data.check_led == '1') {
//         auto_click_on_cho_led.checked = true;
//         var statusElement = document.getElementById('nhietdo1');
//         statusElement.innerText = 'Trạng thái LED dang bat';
//     } else {
//         auto_click_off_cho_led.checked = true;
//         var statusElement = document.getElementById('nhietdo1');
//         statusElement.innerText = 'Trạng thái LED dang tat';
//     }
//     if (data.check_quat == '1') {
//         auto_click_on_cho_quat.checked = true;
//         var statusElement = document.getElementById('quat1');
//         statusElement.innerText = 'Trạng thái Quat dang bat';
//     } else {
//         auto_click_off_cho_quat.checked = true;
//         var statusElement = document.getElementById('quat1');
//         statusElement.innerText = 'Trạng thái Quat dang tat';
//     }
//     if (data.check_bom == '1') {
//         auto_click_on_cho_may_bom.checked = true;
//         var statusElement = document.getElementById('may_bom');
//         statusElement.innerText = 'Trạng thái máy bơm dang bat';
//     } else {
//         auto_click_off_cho_may_bom.checked = true;
//         var statusElement = document.getElementById('may_bom');
//         statusElement.innerText = 'Trạng thái máy bơm dang tat';
//     }
//     var nhietdo = document.getElementById("nguong nhiet do").value;
//     document.getElementById('nguong_nhiet_do').innerText = 'Giá trị ngưỡng hiện tại của nhiệt độ:' + data.nguong_nhiet_do1;
//     // var ADC_do_am = document.getElementById("ADC do am dat").value;
//     document.getElementById('dat').innerText = 'Giá trị ngưỡng hiện tại của ADC do am dat:' + data.nguong_adc_do_am_dat;;
//     // var ADC_quang_tro = document.getElementById("ADCquangtro").value;
//     document.getElementById('quang').innerText = 'Giá trị ngưỡng hiện tại của ADC quang trở:' + data.nguong_adc_quang_tro;
//     // var statusElement = document.getElementById('mode_hoat_dong');
//     // statusElement.innerText = 'Mode hoạt động :' + status;
//     // updateMode('Bang Tay')
// }); <
// /script>



// <
// /body>

// <
// /html>