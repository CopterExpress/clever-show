let animationInput = document.getElementById('animationFile');
let configInput = document.getElementById('configFile');
let arucoInput = document.getElementById('arucoFile');

animationInput.onchange = function (e) {
    document.getElementById('animationFileLabel').innerText = animationInput.files[0].name;
};
configInput.onchange = function (e) {
    document.getElementById('configFileLabel').innerText = configInput.files[0].name;
};
arucoInput.onchange = function (e) {
    document.getElementById('arucoFileLabel').innerText = arucoInput.files[0].name;
};

function sendRows(selectedRows) {
    var animationFile = animationInput.files[0];
    var configFile = configInput.files[0];
    var arucoFile = arucoInput.files[0];
    spinner.style.display = 'inline-block';
    setTimeout(function () {
        selectedRows.forEach(function (element) {
            if (animationFile) {
                let animReq = new XMLHttpRequest();
                let animFormData = new FormData();
                animFormData.append("file", animationFile);
                animReq.open("POST", '/set/animation?ip=' + element._row.data.ip, false);
                animReq.send(animFormData);
            }
            if (configFile) {
                let configReq = new XMLHttpRequest();
                let congifFormData = new FormData();
                congifFormData.append("file", configFile);
                configReq.open("POST", '/set/config?ip=' + element._row.data.ip, false);
                configReq.send(congifFormData);
            }
            if (arucoFile) {
                let arucoReq = new XMLHttpRequest();
                let arucoFormData = new FormData();
                arucoFormData.append("file", arucoFile);
                arucoReq.open("POST", '/set/animation?ip=' + element._row.data.ip, false);
                arucoReq.send(arucoFormData);
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