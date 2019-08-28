let spinner = document.getElementById('spinner');
var tabledata = [];
updateData();

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
        {title: "IP", field: "ip"},
        {title: "Animation id", field: "anim_id"},
        {title: "Batt voltage", field: "batt_voltage"},
        {title: "Cell voltage", field: "cell_voltage"},
        {title: "Selfcheck", field: "selfcheck"},
        {title: "Time", field: "time"},
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