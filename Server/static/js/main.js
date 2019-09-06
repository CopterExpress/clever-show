let spinner = document.getElementById('spinner');
var tabledata = [];
var delay = 0;
const green = "#20E828";
const red = "#FF2F1B";
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
        {
            title: "Animation id", field: "anim_id", formatter: function (cell) {
                let animation = cell.getValue();
                if (animation === "No animation") {
                    cell.getElement().style.background = red;
                } else if (animation) {
                    cell.getElement().style.background = green;
                } else {
                    cell.getElement().style.background = null;
                }
                return animation;
            }
        },
        {
            title: "Batt voltage", field: "batt_voltage", formatter: function (cell) {
                let voltage = cell.getValue();
                if (parseFloat(voltage) > 10) {
                    cell.getElement().style.background = green;
                } else if (voltage) {
                    cell.getElement().style.background = red;
                } else {
                    cell.getElement().style.background = null;
                }
                return voltage;
            }
        },
        {
            title: "Cell voltage", field: "cell_voltage", formatter: function (cell) {
                let cell_voltage = cell.getValue();
                if (parseFloat(cell_voltage) > 3.2) {
                    cell.getElement().style.background = green;
                } else if (cell_voltage) {
                    cell.getElement().style.background = red;
                } else {
                    cell.getElement().style.background = null;
                }
                return cell_voltage;
            }
        },
        {
            title: "Selfcheck", field: "selfcheck", formatter: function (cell) {
                let selfcheck = cell.getValue();
                if (selfcheck === "OK") {
                    cell.getElement().style.background = green;
                } else if (selfcheck) {
                    cell.getElement().style.background = red;
                } else {
                    cell.getElement().style.background = null;
                }
                return selfcheck;
            }
        },
        {
            title: "Time delta", field: "time", formatter: function (cell) {
                let time = cell.getValue();
                if (Math.abs(parseFloat(time)) > 1) {
                    cell.getElement().style.background = red;
                } else if (time) {
                    cell.getElement().style.background = green;
                } else {
                    cell.getElement().style.background = null;
                }
                return time;
            }
        },
    ],
});


function refreshRows(selectedRows) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        selectedRows.forEach(function (element) {
            let req = new XMLHttpRequest();
            req.open('POST', '/selfcheck/selected?ip=' + element._row.data.ip, false);
            req.send();
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
    if (table.getSelectedRows().length > 0) {
        askPermission('Are you sure you want to take off this copters?', function () {
            sendCommandToSelected('takeoff');
        });
    }
}

function flipCopters() {
    if (table.getSelectedRows().length > 0) {
        askPermission('Are you sure you want to flip this copters?', function () {
            sendCommandToSelected('flip');
        });
    }
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
    askPermission('Are you sure you want to start animation?', function () {
        spinner.style.display = 'inline-block';
        setTimeout(function () {
            let req = new XMLHttpRequest();
            req.open('POST', '/start_animation', false);
            req.send();
            spinner.style.display = 'none';
        }, 20);
    });
}

function askPermission(text, func) {
    Ply.dialog("confirm",
        {},
        text
    ).always(function (e) {
        if (e.state) {
            func();
        }
    });
}