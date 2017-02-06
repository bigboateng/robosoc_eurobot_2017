'use strict';

const nodemailer = require('nodemailer');
// var firebase = require('firebase');
var MailListener = require("mail-listener2");
var piServers = ["secondary", "primary"];
var address = require('address');



// create reusable transporter object using the default SMTP transport
let transporter = nodemailer.createTransport({
    service: 'gmail',
    auth: {
        user: 'robosociphost@gmail.com',
        pass: 'robosoc_eurobot'
    }
});


var mailOptions = {
    from: '"Ip Host" <robosociphost@gmail.com>', // sender address
    to: "bigboateng2011@gmail.com", // list of receivers
    subject: 'Message from PI', // Subject line
    text: "Just booted up, IP = " + address.ip()
};

sendEmail(mailOptions);




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
    subject: 'Your requested my IP, Here it is!', // Subject line
    text: address.ip()
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






 

function sendEmail(MailOptions) {
	// send mail with defined transport object
transporter.sendMail(MailOptions, (error, info) => {
    if (error) {
        return console.log(error);
    }
    console.log('Message %s sent: %s', info.messageId, info.response);
});
}
