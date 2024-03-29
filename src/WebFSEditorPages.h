#ifndef WEBFSEDITORPAGES_H
#define WEBFSEDITORPAGES_H

#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html lang="en">
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
    body {
      font-family: Arial, sans-serif;
      padding:1em;
      margin:0;
      font-size:12px;
      background-color: #888;
    }
    #nav {
      background-color:#343a40;
      color:white;
      margin: -12px -12px 8px -12px;
      padding: 8px 12px;
      height: 32px;
    }
    h1 {
      padding-top:10px;
      font-size:1.5em;
      margin:0;
    }
    .nb {
      float:right;
    }
    .container {
      background-color:white;
      border-radius: 6px;
      padding: 8px 8px;
      margin-bottom:8px;
    }
    h2 {
      color:#343a40;
      padding: 4px 8px 4px 0;
      font-size:1.3em;
      margin:0;
    }
    table {
      margin:0;
      border-spacing:0;
    }
    th {
      padding: 4px 8px;
      background-color:#343a40;
      color:white;
    }
    td {
      padding: 4px 8px;
    }
    #info {
      display:none;
    }
    #edittext {
      clear:both;
      display:block;
      box-sizing: border-box;
      width:100%;
      height:400px;
    }
    button {
      background-color:#007bff;
      color:white;
      padding: .375rem .75rem;
      line-height: 1.5;
      border-radius: .25rem;
      border:0;
      cursor:pointer;
    }
    #subnav {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      grid-gap:8px;
    }
    .danger {
      background-color:#dc3545;
    }
    #sb {
      float:right;
      margin-bottom:8px;
    }
    </style>
  </head>
  <body>
    <div id="nav">
      <div class="nb">
        <button onclick="listFilesButton()">List Files</button>
        <button onclick="showUploadButtonFancy()">Upload File</button>
        <button class="danger" onclick="rebootButton()">Reboot</button>
      </div>
      <h1>DroneNode</h1>
    </div>
    <div id="subnav" class="container">
      <button onclick="viewButton('/nodeInfo')">Network Info</button>
      <button onclick="viewButton('/channelInfo')">Channel Info</button>
      <button onclick="viewButton('/modules')">Loaded Modules</button>
      <button onclick="viewButton('/pins')">Pin Assignments</button>
    </div>
    <div id="info" class="container">
      <div id="detailsheader"></div>
      <p id="details"></p>
      <p id="status"></p>
    </div>
    <div class="container">
      <button id="sb" onclick="saveButton()">Save</button>
      <h2 id="edittitle">Editor</h2>
      <textarea id="edittext">...</textarea>
    </div>
  <script>
  var fn = '';
  function rebootButton() {
    document.getElementById("detailsheader").innerHTML = "Rebooting ...";
    var xhr = new XMLHttpRequest();
    //window.open("/execution?execute","_self");
    xhr.open("GET", "/reboot", true);
    xhr.send();
  }
  function listFilesButton() {
    document.getElementById("info").style.display = "block";
    xmlhttp=new XMLHttpRequest();
    xmlhttp.open("GET", "/listfiles", false);
    xmlhttp.send();
    document.getElementById("detailsheader").innerHTML = "<h2>Files</h2>";
    document.getElementById("details").innerHTML = xmlhttp.responseText;
  }
  function downloadDeleteButton(filename, action) {
    var urltocall = "/file?name=" + filename + "&action=" + action;
    xmlhttp=new XMLHttpRequest();
    if (action == "delete") {
      xmlhttp.open("GET", urltocall, false);
      xmlhttp.send();
      document.getElementById("status").innerHTML = xmlhttp.responseText;
      xmlhttp.open("GET", "/listfiles", false);
      xmlhttp.send();
      document.getElementById("details").innerHTML = xmlhttp.responseText;
    }
    if (action == "download") {
      //document.getElementById("status").innerHTML = "";
      //window.open(urltocall,"_blank");
      fn = filename;
      xmlhttp.open("GET", urltocall, false);
      xmlhttp.send();
      document.getElementById("edittext").value =xmlhttp.responseText;
      document.getElementById("edittitle").innerHTML = filename;
      document.getElementById("sb").style.display = "block";
    }
  }
  function showUploadButtonFancy() {
    document.getElementById("detailsheader").innerHTML = "<h2>Upload File</h2>";
    document.getElementById("status").innerHTML = "";
    var uploadform = "<form method = \"POST\" action = \"/\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"data\"/><input type=\"submit\" name=\"upload\" value=\"Upload\" title = \"Upload File\"></form>"
    document.getElementById("details").innerHTML = uploadform;
    var uploadform =
    "<form id=\"upload_form\" enctype=\"multipart/form-data\" method=\"post\">" +
    "<input type=\"file\" name=\"file1\" id=\"file1\" onchange=\"uploadFile()\"><br>" +
    "<progress id=\"progressBar\" value=\"0\" max=\"100\" style=\"width:300px;\"></progress>" +
    "<h3 id=\"status\"></h3>" +
    "<p id=\"loaded_n_total\"></p>" +
    "</form>";
    document.getElementById("details").innerHTML = uploadform;
  }
  function _(el) {
    return document.getElementById(el);
  }
  function uploadFile() {
    var file = _("file1").files[0];
    // alert(file.name+" | "+file.size+" | "+file.type);
    var formdata = new FormData();
    formdata.append("file1", file);
    var ajax = new XMLHttpRequest();
    ajax.upload.addEventListener("progress", progressHandler, false);
    ajax.addEventListener("load", completeHandler, false); // doesnt appear to ever get called even upon success
    ajax.addEventListener("error", errorHandler, false);
    ajax.addEventListener("abort", abortHandler, false);
    ajax.open("POST", "/");
    ajax.send(formdata);
  }
  function progressHandler(event) {
    //_("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes of " + event.total; // event.total doesnt show accurate total file size
    _("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes";
    var percent = (event.loaded / event.total) * 100;
    _("progressBar").value = Math.round(percent);
    _("status").innerHTML = Math.round(percent) + "% uploaded... please wait";
    if (percent >= 100) {
      _("status").innerHTML = "Please wait, writing file to filesystem";
    }
  }
  function completeHandler(event) {
    _("status").innerHTML = "Upload Complete";
    _("progressBar").value = 0;
    xmlhttp=new XMLHttpRequest();
    xmlhttp.open("GET", "/listfiles", false);
    xmlhttp.send();
    document.getElementById("status").innerHTML = "File Uploaded";
    document.getElementById("detailsheader").innerHTML = "<h2>Files</h2>";
    document.getElementById("details").innerHTML = xmlhttp.responseText;
  }
  function errorHandler(event) {
    _("status").innerHTML = "Upload Failed";
  }
  function abortHandler(event) {
    _("status").innerHTML = "inUpload Aborted";
  }
  function viewButton(path) {
    document.getElementById("sb").style.display = "none";
    xmlhttp=new XMLHttpRequest();
    fn = path;
    xmlhttp.open("GET", path, false);
    xmlhttp.send();
    document.getElementById("edittext").value =xmlhttp.responseText;
    document.getElementById("edittitle").innerHTML = fn;
  }
  function saveButton() {
    document.getElementById("details").innerHTML = "...";
    var contents = document.getElementById("edittext").value;
    var blob = new Blob ([contents], { type: "text/plain" });
    var fileOfBlob = new File([blob], fn);
    var fd = new FormData();
    fd.append("file1", fileOfBlob);
    xmlhttp=new XMLHttpRequest();
    xmlhttp.open("POST", "/", false);
    xmlhttp.send(fd);
    setTimeout(listFilesButton, 1000);
  }
  </script>
  </body>
  </html>

)rawliteral";

const char logout_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
</head>
<body>
  <p><a href="/">Log Back In</a></p>
</body>
</html>
)rawliteral";


// reboot.html base upon https://gist.github.com/Joel-James/62d98e8cb3a1b6b05102
const char reboot_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta charset="UTF-8">
</head>
<body>
<h3>
  Rebooting, returning to main page in <span id="countdown">30</span> seconds
</h3>
<script type="text/javascript">
  var seconds = 20;
  function countdown() {
    seconds = seconds - 1;
    if (seconds < 0) {
      window.location = "/";
    } else {
      document.getElementById("countdown").innerHTML = seconds;
      window.setTimeout("countdown()", 1000);
    }
  }
  countdown();
</script>
</body>
</html>
)rawliteral";

#endif
