let spinner = document.getElementById('spinner');
var tabledata = [];
var delay = 0;
updateData();
updateDelay();

function updateDelay() {
    let req = new XMLHttpRequest();
    req.open('POST', '/get/delay', false);
    req.send();
    delay = parseInt(req.response);
    document.getElementById('delay').placeholder = 'Set delay (now is ' + delay.toString() + ')';
    document.getElementById('delay').value = null;
}

function setDelay(delay) {
    let req = new XMLHttpRequest();
    req.open('POST', '/set/delay?delay=' + delay.toString(), false);
    req.send();
}

function updateData() {
    let req = new XMLHttpRequest();
    req.open('POST', '/selfcheck/all', false);
    req.send();
    tabledata = JSON.parse(req.response);
}

var table = new Tabulator("#copters-table", {
    data: tabledata,
    reactiveData: true,
    selectable: true,
    layout: "fitColumns",
    columns: [
        {title: "Name", field: "name"},
        {title: "Animation id", field: "anim_id"},
        {title: "Batt voltage", field: "batt_voltage"},
        {title: "Cell voltage", field: "cell_voltage"},
        {title: "Selfcheck", field: "selfcheck"},
        {title: "Time delta", field: "time"},
    ],
});

function refreshRows(selectedRows) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        selectedRows.forEach(function (element) {
            let req = new XMLHttpRequest();
            req.open('POST', '/selfcheck/selected?ip=' + element._row.data.ip, false);
            req.send();
            element.deselect();
            let response = JSON.parse(req.response);
            Object.keys(response).forEach(function (item) {
                element._row.data[item] = response[item];
            });
        });
        spinner.style.display = 'none';
    }, 20);
}

function refreshSelected() {
    refreshRows(table.getSelectedRows());
}

function refreshAll() {
    refreshRows(table.getRows());
}

function selectAll() {
    table.getRows().forEach(function (element) {
        element.select();
    });
}

function deselectAll() {
    table.getRows().forEach(function (element) {
        element.deselect();
    });
}

function sendCommandToSelected(command) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        table.getSelectedRows().forEach(function (element) {
            let req = new XMLHttpRequest();
            req.open('POST', '/' + command + '/selected?ip=' + element._row.data.ip);
            req.send();
            element.deselect();
        });
        spinner.style.display = 'none';
    }, 20);
}

function testLedSelected() {
    sendCommandToSelected('test_led');
}

function stopCopters() {
    let req = new XMLHttpRequest();
    req.open('POST', '/stop/all', false);
    req.send();
}

function setStartTime() {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        setDelay(parseInt(document.getElementById('delay').value));
        updateDelay();
        spinner.style.display = 'none';
    }, 20);
}

function takeOff() {
    sendCommandToSelected('takeoff');
}

function flipCopters() {
    sendCommandToSelected('flip');
}

function land() {
    let req = new XMLHttpRequest();
    req.open('POST', '/land/all', false);
    req.send();
}

function disarm() {
    let req = new XMLHttpRequest();
    req.open('POST', '/disarm/all', false);
    req.send();
}

function pauseCopters() {
    sendCommandToSelected('pause');
}

function resumeCopters() {
    sendCommandToSelected('resume');
}

function restart_fcu() {
    sendCommandToSelected('reboot_fcu');
}

function calibrate_gyro() {
    sendCommandToSelected('calibrate_gyro');
}

function calibrate_level() {
    sendCommandToSelected('calibrate_level');
}

function emLand() {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        let req = new XMLHttpRequest();
        req.open('POST', '/em_land', false);
        req.send();
        spinner.style.display = 'none';
    }, 20);
}

function startAnimation() {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        let req = new XMLHttpRequest();
        req.open('POST', '/start_animation', false);
        req.send();
        spinner.style.display = 'none';
    }, 20);
}