#define QUOTE(...) #__VA_ARGS__   /// very nice trick, forever thankfull to the guys at https://stackoverflow.com/questions/797318/how-to-split-a-string-literal-across-multiple-lines-in-c-objective-c
const char *indexStr = QUOTE(
<!DOCTYPE html>
<html>
<head>
    <title>Water Reservoir Status</title>
    <meta charset=utf-8>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="manifest" href='data:application/manifest+json,{"name":"Rezervoar vode","short_name":"Rezervoar vode","description":"Webapp za praćenje stanja rezervoara vode","background":"rgb(255,255,255)","display":"standalone"}'/>
    <style>
        body {
            background-color: #f2f2f2;
        }
        .container {
            background-color: rgb(246, 246, 246);
            /*margin: 20px 20px 20px 20px;*/
            margin: 0px auto;
            padding: 5px 20px 20px 20px;
            max-width: 600px;
            border-radius: 10px;
            font-family: Tahoma, Geneva, Verdana, sans-serif;
            text-align: center;
        }
        .values-container {
            margin: 0px auto;
            padding: 0px 20px 0px 20px;
            max-width: 600px;
            font-family: Tahoma, Geneva, Verdana, sans-serif;
            text-align: center;
        }
        .reservoir-container {
            margin: 0 auto;
            justify-content: center;
            align-items: center;
            position: relative;
            width: 60%;
            height: 40vh;
            margin-top: 20px;
            margin-bottom: 20px;
            border-radius: 5px;
            background-color: #ddd;
        }
        .water-level {
            position: absolute;
            bottom: 0;
            width: 100%;
            height: var(--water-percentage);
            background-color: #1e90ff;
            border-radius: 0 0 5px 5px;
        }
        .valve {
            position: relative;
            bottom: 10%;
            left: calc(50% - 10%);
            aspect-ratio : 1 / 1;
            width: 20%;
            border-radius: 50%;
            background-color: var(--valve-color);
            display: flex;
            justify-content: center;
            align-items: center;
            font-size: 80%;
            color: #fff;
        }
        .valve2 {
            position: relative;
            bottom: +50%;
            left: calc(100% - 10%);
            aspect-ratio : 1 / 1;
            width: 20%;
            border-radius: 50%;
            background-color: var(--valve-color);
            display: flex;
            justify-content: center;
            align-items: center;
            font-size: 80%;
            color: #fff;
        }
        .row {
      display: flex;
      flex-direction: row;
      justify-content: space-between;
      align-items: center;
      padding: 8px 0;
    }
    .label {
      width: 50%;
      font-weight: bold;
      text-align: left;
    }
    .value {
      width: 25%;
      text-align: right;
    }
    .unit {
      float: right;
      width: 25%;
      text-align: right;
      padding-left: 5px;
    }
        .reservoir-percentage {
            position: relative;
            font-size: 500%;
            font-weight: bold;
            margin: auto auto;
            color: rgb(40, 40, 40);;
            bottom: 110%;
        }
        .triangle-up{
            position: relative;
            margin: 5% auto;
            width: 0;
            height: 0;
            border-left: 2em solid transparent;
            border-right: 2em solid transparent;
            border-bottom: 2em solid rgb(40, 40, 40);
            bottom: 100%;
            display: none;
        }
        .triangle-down{
            position:relative;
            margin:5% auto;
            width:0;
            height:0;
            border-left:2em solid transparent;
            border-right:2em solid transparent;
            border-top:2em solid rgb(40, 40, 40);
            bottom:100%;
            display:none;
        }
        .reservoir{
            position: relative;
            margin: 0% auto;
            height: 50vh;
            aspect-ratio : 1 / 1.5;
        }
        .circle {
  position: relative;
  margin: 5% auto;
  height: 50vh;
  aspect-ratio : 1 / 1.5;
  background-color: rgb(210, 210, 210);
  border: 2px solid rgb(80, 80, 80);
  border-radius: 2%;
  overflow: hidden;
  -webkit-backface-visibility: hidden;
  -moz-backface-visibility: hidden;
  -webkit-transform: translate3d(0, 0, 0);
  -moz-transform: translate3d(0, 0, 0);
  cursor:pointer;
}
.wave {
  background-color: #3f68c5;
  position: relative;
  margin: auto auto;
  top: 105%;
  height: 800%;
  aspect-ratio : 1 / 1;
  border-radius: 49.7%;
  left: -550%;
  transition: all 5s ease-in-out;
  animation: wave 5s infinite linear;
  -webkit-animation-fill-mode: forwards; /* Chrome 16+, Safari 4+ */
  -moz-animation-fill-mode: forwards;    /* FF 5+ */
  -o-animation-fill-mode: forwards;      /* Not implemented yet */
  -ms-animation-fill-mode: forwards;     /* IE 10+ */
  animation-fill-mode: forwards;         /* When the spec is finished */
}
@keyframes wave {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}    
.dev-offline{
    position: relative;
    margin: 2%;
    padding: 2%;
    bottom: 105%;
    background: repeating-linear-gradient(45deg,#606cbc00,#606cbc00 10px,#ffcf22 10px,#ffcf22 20px);
    display: block;
}
a {
  color: blue;
  text-decoration: none; /* no underline */
}
    </style>
</head>
<body>
    <div class="container">
        <div class="reservoir">
            <div class="circle">
                <div id="water" class="wave"></div>
            </div>
            <div class="valve" style="--valve-color: #7f8c8d">
                <span id="valve-status">---</span>
            </div>
            <div class="valve2" style="--valve-color: #7f8c8d">
                <span id="valve2-status">|</span>
            </div>
            <div class="reservoir-percentage" id="water-percentage">?</div>
            <div class="dev-offline">SENZOR NEDOSTUPAN</div>
            <div class="triangle-down"></div>
            <div class="triangle-up"></div>
        </div>
    </div>
    <div class="values-container">
        <div class="row">
      <div class="label">Napunjenost:</div>
      <div id="water-percentext" class="value">0</div>
      <div class="unit">[ % ]</div>
    </div>
    <div class="row">
      <div class="label">Litri u rezervoaru:</div>
      <div id="liters-text" class="value">0</div>
      <div class="unit">[ L ]</div>
    </div>
        <div class="row">
      <div id="timeUntilLabel" class="label">Rezervoar prazan za:</div>
      <div id="time-until" class="value">0:0:0</div>
      <div class="unit">[ H:M:S ]</div>
    </div>
    <div class="row">
      <div class="label">Protok:</div>
      <div id="flow" class="value">0</div>
      <div class="unit">[ L/min ]</div>
    </div>
    <div class="row">
      <div class="label">Prosečan protok pražnjenja:</div>
      <div id="awg-outflow" class="value">0</div>
      <div class="unit">[ L/min ]</div>
    </div>
        <div class="row">
      <div class="label">Prosečan protok punenja:</div>
      <div id="awg-inflow" class="value">0</div>
      <div class="unit">[ L/min ]</div>
    </div>
    <div class="row">
      <div class="label">Merena količina:</div>
      <div id="measured" class="value">0</div>
      <div class="unit">[ L ]</div>
    </div>
    <div class="row">
      <div class="label">Total količina:</div>
      <div id="total" class="value">0</div>
      <div class="unit">[ L ]</div>
    </div>
    <div class="row">
      <div class="label">Napon baterije:</div>
      <div id="voltage" class="value">0</div>
      <div class="unit">[ V ]</div>
    </div>
        <br><br>
        <div class="row">
      <div class="label">Ažurirano:</div>
      <div id="refreshed" class="value">00:00:00</div>
      <div class="unit">[ H:M:S ]</div>
    </div>
        <div class="row">
      <div class="label">Merena visina:</div>
      <div id="sensorSubmerged" class="value">0</div>
      <div class="unit">[ mm ]</div>
    </div>
        <div class="row">
      <div class="label">Senzor</div>
      <div id="freqDbg" class="value">0</div>
      <div class="unit">[ br ]</div>
    </div>
        <div class="row">
      <div class="label">IP adresa:</div>
            <a href="http://192.168.4.1"><div id="ipAddr" class="value">192.168.4.1</div></a>
      <div class="unit"></div>
    </div>
    </div>
    <script>
        // Update valve status and water level
        const inValve = document.querySelector('.valve2');
        const outValve = document.querySelector('.valve');
        const arrowUp = document.querySelector('.triangle-up');
        const arrowDown = document.querySelector('.triangle-down');
        const waterLevel = document.querySelector('.water-level');
        const offlineFlag = document.querySelector('.dev-offline');
        var level = 0,liters = 0,timeUntil="00:00:00",flow=0,outflow=0,inflow=0,measured=0,total=0,voltage=0, ipAddresa = "", freqDbg=0, sensorSubmerged = 0;
        var offlineCounter = 0;

    function nFormatter(num, digits) {
        const lookup = [
        {value: 1,symbol:""},
        {value:1e3,symbol:"K"},
        {value:1e6,symbol:"M"},
        {value:1e9,symbol:"G"},
        {value:1e12,symbol:"T"},
        {value:1e15,symbol:"P"},
        {value:1e18,symbol:"E"}
        ];
        const rx = /\.0+$|(\.[0-9]*[1-9])0+$/;
        var item = lookup.slice().reverse().find(function(item) {
        return num >= item.value;
        });
    return item ? (num / item.value).toFixed(digits).replace(rx, "$1") + item.symbol : "0";
    }

    setInterval(function ( ) {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            offlineCounter = 0;
            var rawStr = this.responseText; //"21.5/2300/00:12:12/0/23/43/1032345/15425345/12.85";//
            const infoArray = rawStr.split("/");
            level = parseFloat(infoArray[0]);//120;
            liters = parseFloat(infoArray[1]);//1200;
            timeUntil = infoArray[2];
            flow = parseFloat(infoArray[3]);
            outflow = parseFloat(infoArray[4]);
            inflow = parseFloat(infoArray[5]);
            measured = parseInt(infoArray[6]);
            total = parseInt(infoArray[7]);
            voltage = parseFloat(infoArray[8]);
            ipAddresa = infoArray[9];
            freqDbg = parseInt(infoArray[10]);
            sensorSubmerged = parseInt(infoArray[11]);

            arrowUp.style.display = "none";
            arrowDown.style.display = "none";

            offlineFlag.style.display = "none";

            if( flow < 20 && flow > -20){
                inValve.innerHTML = '|';
                inValve.style.setProperty('--valve-color', '#E43D3D');
                outValve.innerHTML = '---';
                outValve.style.setProperty('--valve-color', '#E43D3D');
                document.getElementById("timeUntilLabel").innerHTML = 'Rezervoar prazan za:';
            }
            else if(flow < 0){
                inValve.innerHTML = '?';
                inValve.style.setProperty('--valve-color', '#7f8c8d');
                outValve.innerHTML = '|';
                outValve.style.setProperty('--valve-color', '#54E862');
                arrowDown.style.display = "block";
                document.getElementById("timeUntilLabel").innerHTML = 'Rezervoar prazan za:';
            }
            else{
                inValve.innerHTML = '---';
                inValve.style.setProperty('--valve-color', '#54E862');
                outValve.style.setProperty('--valve-color', '#7f8c8d');
                outValve.innerHTML = '?';
                arrowUp.style.display = "block";
                document.getElementById("timeUntilLabel").innerHTML = 'Rezervoar pun za:';
            }
            document.querySelector('#water-percentage').innerText = Math.min(Math.max(level, 0), 99.9).toFixed(1);
            document.getElementById('water').style.borderRadius = '49.7%';
            document.getElementById('water').style.top = (100 - level.toFixed(1)) + '%';
            setTimeout(function(){
                document.getElementById('water').style.borderRadius = '49.9%';
            }, 5000);
            document.getElementById('water-percentext').innerHTML = level.toFixed(1);
            document.getElementById('liters-text').innerHTML = liters;
            document.getElementById('time-until').innerHTML = timeUntil;
            document.getElementById('flow').innerHTML = flow;
            document.getElementById('awg-outflow').innerHTML = outflow;
            document.getElementById('awg-inflow').innerHTML = inflow;
            document.getElementById('measured').innerHTML = nFormatter(measured, 2);
            document.getElementById('total').innerHTML = nFormatter(total, 2);
            document.getElementById('voltage').innerHTML = voltage;
            document.getElementById('refreshed').innerHTML = (new Date()).toLocaleTimeString('hr-HR');
            document.getElementById('ipAddr').innerHTML = ipAddresa;
            document.getElementById('sensorSubmerged').innerHTML = sensorSubmerged;
            document.getElementById('freqDbg').innerHTML = freqDbg;
            
      
        }
        else{
            offlineCounter = offlineCounter + 1;
            if(offlineCounter > 6){
                offlineFlag.style.display = "block";
            }
        }
        };
    xhttp.open("GET", "/info", true);
    xhttp.send();
    }, 5500 ) ;
    </script>
    </body>
    </html>
);
