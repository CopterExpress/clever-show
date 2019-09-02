let animationInput = document.getElementById('animationFile');
let configInput = document.getElementById('configFile');
let arucoInput = document.getElementById('arucoFile');

animationInput.onchange = function (e) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        if (animationInput.files.length > 0) {
            let fileReq = new XMLHttpRequest();
            let fileFormData = new FormData();
            for (let i = 0; i < animationInput.files.length; i++) {
                fileFormData.append(animationInput.files[i].name, animationInput.files[i]);
            }
            fileReq.open("POST", '/set/animation', false);
            fileReq.send(fileFormData);
            deselectAll();
            spinner.style.display = 'none';
        }
    }, 20);
};

configInput.onchange = function (e) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        if (configInput.files.length > 0) {
            let fileReq = new XMLHttpRequest();
            let fileFormData = new FormData();
            for (let i = 0; i < configInput.files.length; i++) {
                fileFormData.append(configInput.files[i].name, configInput.files[i]);
            }
            let ips = [];
            table.getSelectedRows().forEach(function (element) {
                ips.push(element._row.data.ip);
            });
            fileFormData.append('ips', ips);
            fileReq.open("POST", '/set/config', false);
            fileReq.send(fileFormData);
            deselectAll();
            spinner.style.display = 'none';
        }
    }, 20);
};

arucoInput.onchange = function (e) {
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        if (arucoInput.files.length > 0) {
            let fileReq = new XMLHttpRequest();
            let fileFormData = new FormData();
            fileFormData.append(arucoInput.files[0].name, arucoInput.files[0]);
            fileReq.open("POST", '/set/aruco', false);
            fileReq.send(fileFormData);
            deselectAll();
            spinner.style.display = 'none';
        }
    }, 20);
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