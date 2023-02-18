#define QUOTE(...) #__VA_ARGS__   /// very nice trick, forever thankfull to the guys at https://stackoverflow.com/questions/797318/how-to-split-a-string-literal-across-multiple-lines-in-c-objective-c
const char *indexStr = QUOTE(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <style>
    body {
      min-width: 310px;
      max-width: 800px;
      height: 400px;
      margin: 0 auto;
      white-space: pre-line;
    }
    h2 {
      font-family: Arial;
      font-size: 2.5rem;
      text-align: center;
    }
    .column {
      float: center;
      width: 80%;
    }
    p{
      font-family: "Times New Roman", Times, serif;
      font-Family: "Courier New", Courier, Arial, Helvetica, sans-serif;
      font-size:16px;
    }

/* Clear floats after the columns */
    .row:after {
      content: "";
      display: table;
      clear: both;
    }
  </style>
</head>
<body>
  <h2></h2>
    <div class="column">
      <p><b>CURRENT DATA:</b></p>
      <p id="tankLevel"></p>
      <p id="tankLiters"></p>
      </br>
      <p id="flow"></p>
      <p id="TOE"></p>
      </br>
      <p><b>STATISTICS:</b></p>
      <p id="totalWater"></p>
      <p id="averageFlow"></p>
      <p id="lastFillTime"></p>
      <p id="fillEnd"></p>
      <p id="firstOpen"></p>
      <p id="lastRefresh"></p>
    </div>
  <div id="chart-temperature" class="container"></div>
</body>
<script>
/*var chartT = new Highcharts.Chart({
  chart:{ renderTo : 'chart-temperature' },
  title: { text: 'Water level' },
  series: [{
    showInLegend: false,
    data: []
  }],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
    series: { color: '#059e8a' }
  },
  xAxis: { type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  yAxis: {
    title: { text: 'Water level [ l ]' }
  },
  credits: { enabled: false }
});*/

/*setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var x = (new Date()).getTime(),
          y = parseFloat(this.responseText);
      if(chartT.series[0].data.length > 300) {
        chartT.series[0].addPoint([x, y], true, true, true);
      } else {
        chartT.series[0].addPoint([x, y], true, false, true);
      }
    }
  };
  xhttp.open("GET", "/currentLevel", true);
  xhttp.send();
}, 3000 ) ;*/
function sexToTmStr(sexStr) {
  var sex = parseInt(sexStr);
  hours = ~~(sex/3600);
  
  mins = ~~((sex%3600)/60);
  sex %= 60;
  return hours + ":" + mins + ":" + sex;
}
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var now = (new Date()).getTime();
      var rawStr = this.responseText;
      const infoArray = rawStr.split("/");
      const labelPadLength = 18;
      const valPadLength = 10;
      document.getElementById("tankLevel").innerHTML = "TANK LEVEL:".padEnd(labelPadLength, '_') + infoArray[0].padStart(valPadLength, '_') + " [   %   ]";
      document.getElementById("tankLiters").innerHTML = "TANK LITERS:".padEnd(labelPadLength, "_") + infoArray[1].padStart(valPadLength, '_') + " [   l   ]";
      document.getElementById("flow").innerHTML = "FLOW RATE: ".padEnd(labelPadLength, '_') + infoArray[2].padStart(valPadLength, '_') + " [l/min]";
      document.getElementById("TOE").innerHTML = "EMPTY OR FULL IN:".padEnd(labelPadLength, '_') + infoArray[3].padStart(valPadLength, '_') + " [  min  ]";
      document.getElementById("totalWater").innerHTML = "TOTAL OUT:".padEnd(labelPadLength, '_') + infoArray[4].padStart(valPadLength, '_') + " [   l   ]";
      document.getElementById("averageFlow").innerHTML = "AVERAGE OPEN FLOW:".padEnd(labelPadLength, '_') + infoArray[5].padStart(valPadLength, '_') + " [l/min]";
      document.getElementById("firstOpen").innerHTML = "OUT OPENED AT:".padEnd(labelPadLength, '_') + sexToTmStr(infoArray[6]).padStart(valPadLength, '_') + " [h:m:s]";
      document.getElementById("fillEnd").innerHTML = "TANK FILLED AT:".padEnd(labelPadLength, '_') + sexToTmStr(infoArray[7]).padStart(valPadLength, '_') + " [h:m:s]";
      document.getElementById("lastFillTime").innerHTML = "LAST FILL TIME:".padEnd(labelPadLength, '_') + infoArray[8].padStart(valPadLength, '_') + " [  min  ]";
      document.getElementById("lastRefresh").innerHTML = "LAST REFRESH:".padEnd(labelPadLength, '_') + (new Date()).toLocaleTimeString('hr-HR').padStart(valPadLength, '_') + " [h:m:s]";

    }
  };
  xhttp.open("GET", "/info", true);
  xhttp.send();
}, 3000 ) ;
</script>
</html>

);
