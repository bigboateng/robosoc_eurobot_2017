'use strict';
var ipMon = require('ip-monitor');
const nodemailer = require('nodemailer');
// var firebase = require('firebase');
var MailListener = require("mail-listener2");
var piServers = ["secondary", "primary"];


var newIP;
// var config = {
//     apiKey: "AIzaSyCRhAY9cctucFTZfPT33X2aKSTQyulpeME",
//     authDomain: "eurobot-a9044.firebaseapp.com",
//     databaseURL: "https://eurobot-a9044.firebaseio.com",
//     storageBucket: "eurobot-a9044.appspot.com",
//     messagingSenderId: "1043814359975"
//   };

// // initialize firebase
// firebase.initializeApp(config);

// create reusable transporter object using the default SMTP transport
let transporter = nodemailer.createTransport({
    service: 'gmail',
    auth: {
        user: 'robosociphost@gmail.com',
        pass: 'robosoc_eurobot'
    }
});


var mailListener = new MailListener({
  username: "robosociphost@gmail.com",
  password: "robosoc_eurobot", // works for me: https://accounts.google.com/b/0/IssuedAuthSubTokens?hide_authsub=1
  host: "imap.gmail.com",
  port: 993, // imap port
  tls: true,
   connTimeout: 10000, // Default by node-imap
  authTimeout: 5000, // Default by node-imap,
  fetchUnreadOnStart: false, //,
  markSeen: true // all fetched email willbe marked as seen and not fetched next time
});

mailListener.start(); // start listening 
 
mailListener.on("mail", function(mail, seqno, attributes){
  // do something with mail object including attachments 
  //console.log("emailParsed", mail);
  var sender = mail.from[0].address;

  var mailOptions = {
    from: '"Ip Host" <robosociphost@gmail.com>', // sender address
    to: sender, // list of receivers
    subject: 'Current Ip Address', // Subject line
    text: newIP
};

sendEmail(mailOptions);
});

mailListener.on("server:connected", function(){
  console.log("imapConnected");
});
 
mailListener.on("server:disconnected", function(){
  console.log("imapDisconnected");
});
 
mailListener.on("error", function(err){
  console.log(err);
});






// Get a reference to the database service
//var database = firebase.database();

 
var watcher = ipMon.createWatcher();
 
watcher.on('IP:change', function (prevIP, currentIp) {
    console.log('Prev IP: %s, New IP: %s', prevIP, currentIp);
    // setup email data with unicode symbols
    newIP = currentIp;
   var mailOptions = {
    from: '"Ip Host" <robosociphost@gmail.com>', // sender address
    to: 'bigboateng2011@gmail.com', // list of receivers
    subject: 'Ip adrress changed', // Subject line
    text: newIP
};

    //sendEmail(mailOptions);
});
 

function sendEmail(MailOptions) {
	// send mail with defined transport object
transporter.sendMail(MailOptions, (error, info) => {
    if (error) {
        return console.log(error);
    }
    console.log('Message %s sent: %s', info.messageId, info.response);
});
}
/*
Generic error event
*/
watcher.on('error', function (error) {
    throw error;
});
 
/*
Seperate event for ip error handling.
It will fire when the connection has been lost, e.g your router is restarting,
thats why you may want to handle it differently than regular errors.
*/
watcher.on('IP:error', function (error) {
    console.log('Cant get external IP: ' + error);
});
 
watcher.on('IP:success', function (IP) {
    console.log('Got IP: %s', IP);
});
 
 
watcher.start();
