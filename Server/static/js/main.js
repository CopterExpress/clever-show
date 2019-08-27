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
