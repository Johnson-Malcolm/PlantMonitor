function idk() {
  alert("Hello world");   // The function returns the product of p1 and p2
}

function init(data)
{
  fillmoisture(data);
  graphTemp(data);
  graphWater(data);
}

//Moisture Table
function fillmoisture(data) {

    var list = data.plantData;
    var body = document.getElementById('moistureTableDiv'),
        tbl  = document.getElementById('moistureTable');
    tbl.style.width  = '100px';
    var th = tbl.createTHead();
    var row = th.insertRow(0);
    var cell = row.insertCell(0);
    cell.innerHTML = "<b>Date</b>";
    var cell1 = row.insertCell(1);
    cell1.innerHTML = "<b>Moisture</b>";
    var dateList = [];
    var moistureList = [];
    for(var i = list.length-1; i > list.length-6; i--)
    {
      dateList.push(data.plantData[i].date);
      moistureList.push(data.plantData[i].moisture);
    }
    for(var i = 0; i < 5; i++){
        var tr = tbl.insertRow();
        for(var j = 0; j < 2; j++){
            var td = tr.insertCell();
            if(j == 0){
              td.appendChild(document.createTextNode(dateList[i]));
            }else{
              td.appendChild(document.createTextNode(moistureList[i]));
            }


        }
    }
    body.appendChild(tbl);

}

//Temp Graph
function graphTemp(data){
  var list = data.plantData;
  var dateList = [];
  var tempList = [];
  for(var i = list.length-1; i > 0; i--)
  {
    dateList.push(data.plantData[i].date);
    tempList.push(data.plantData[i].temperature);
    console.log(tempList[i]);
  }

  for(var j = 0; j > tempList.length; j++)
  {
     dict.push({label:dateList[j] , y:tempList[j]});
   }

  var chart = new CanvasJS.Chart("chartContainer", {

     title:{
      text: "Temperature (F˚) Over Time"
      },
       data: [
      {
        type: "line",

        dataPoints: [
            { label: dateList[0], y: tempList[0]},
            { label: dateList[1], y: tempList[1]},
            { label: dateList[2], y: tempList[2]},
            { label: dateList[3], y: tempList[3]},
            { label: dateList[4], y: tempList[4]},
            { label: dateList[5], y: tempList[5]},
            { label: dateList[6], y: tempList[6]},
            { label: dateList[7], y: tempList[7]},
            { label: dateList[8], y: tempList[8]}
          ]
      }
      ]
    });

    chart.render();

}

//Water Consumption Chart
function graphWater(data){
  var list = data.plantData;
  var dateList = [];
  var waterList = [];
  var waterTally = 0;
  for(var i = list.length-1; i > 0; i--)
  {
    dateList.push(data.plantData[i].date);
    waterList.push(data.plantData[i].water);
    console.log(data.plantData[i].water);
    waterTally += parseInt(data.plantData[i].water);
  }


  var chart = new CanvasJS.Chart("chartContainer1", {

     title:{
      text: "Water Consumption (mL) Over Time"
      },
      responsive: true,
      aspectRatio: 3,
      theme: "light2",
       data: [
      {
        type: "column",

        dataPoints: [
            { label: dateList[0], y: waterList[0]},
            { label: dateList[1], y: waterList[1]},
            { label: dateList[2], y: waterList[2]},
            { label: dateList[3], y: waterList[3]},
            { label: dateList[4], y: waterList[4]},
            { label: dateList[5], y: waterList[5]},
            { label: dateList[6], y: waterList[6]},
            { label: dateList[7], y: waterList[7]},
            { label: dateList[8], y: waterList[8]},
            { label: dateList[9], y: waterList[9]}
          ]
      }
      ]
    });

    chart.render();
    var str = waterTally.toString() + " mL";
    document.getElementById("waterVar").innerHTML = str;
}
