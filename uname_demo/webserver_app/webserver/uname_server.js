#!/usr/bin/env node

/*
 * Libraries
 */
const express = require('express');
const { execFile } = require('child_process');
const serveStatic = require('serve-static');
const path = require('path');
const http = require('http');

/*
 * Definitions
 */

/* Set the path */
const demo_dir = process.env.APP_DIR || path.join(__dirname, '../../webserver_app/app');

/* Create the express app */
const app = express();
const server = http.createServer(app);

/*
 * Server Code
 */

/* Place the GUI files onto the server and point to the main html file */
app.use(serveStatic(demo_dir, {'index': ['index.html']}));

/* Handle incoming GET requests to run uname */
app.get("/run-uname", (request, response) => {
  const scriptPath = path.join(__dirname, '../../webserver_app/linux_app/run_uname.sh');
  execFile(scriptPath, (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      response.status(500).send(stderr);
      return;
    }
    response.send(stdout);
  });
});

/* Open the server on port 8081 */
server.listen(8081, () => {
  console.log('Server is running on http://localhost:8081');
});
