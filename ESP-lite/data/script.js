const compassSlider = document.getElementById('compassSlider');

if (compassSlider) {
    const middleValue = 0;
    compassSlider.value = middleValue;
    updateCompass(middleValue);
}

function updateCompass(pos) {
    document.getElementById('compassValue').innerText = `${pos}°`;
}

function sendCompassValue(pos) {
    fetch(`/compass?value=${pos}`);
    console.log("Compass value", pos);
}

function findNorth() {
    fetch(`/compass?value=FindNorth`);  // 修改这里，使用 compass 路由发送 FindNorth 命令
    console.log("Finding North");
}

function forwards5() { move('forwards', 5); }
function forwards20() { move('forwards', 20); }
function backwards5() { move('backwards', 5); }
function backwards20() { move('backwards', 20); }

function move(dir, dis) {
    fetch(`/${dir}${dis}`);
    console.log("Drive", dir, dis);
}