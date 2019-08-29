let animationInput = document.getElementById('animationFile');
let configInput = document.getElementById('configFile');
let arucoInput = document.getElementById('arucoFile');

animationInput.onchange = function (e) {
    sendRows(table.getSelectedRows(), animationInput.files[0], 'animation');
};
configInput.onchange = function (e) {
    sendRows(table.getSelectedRows(), configInput.files[0], 'config');
};
arucoInput.onchange = function (e) {
    sendRows(table.getSelectedRows(), arucoInput.files[0], 'aruco');
};

function sendRows(selectedRows, file, file_type) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        selectedRows.forEach(function (element) {
            if (file) {
                let fileReq = new XMLHttpRequest();
                let fileFormData = new FormData();
                fileFormData.append("file", file);
                fileReq.open("POST", '/set/' + file_type + '?ip=' + element._row.data.ip, false);
                fileReq.send(fileFormData);
            }
            element.deselect();
        });
        spinner.style.display = 'none';
    }, 20);
}

function sendSelected() {
    sendRows(table.getSelectedRows());
}

function sendAll() {
    sendRows(table.getRows());
}