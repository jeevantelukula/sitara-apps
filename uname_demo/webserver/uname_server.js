#!/usr/bin/env node

/*
 * Libraries
 */
const express = require('express');
const { execFile } = require('child_process');
const serveStatic = require('serve-static');

/*
 * Definitions
 */

/* Set the path */
const demo_dir = process.argv[2];

/* Create the express app */
const app = express();

/*
 * Server Code
 */

/* Place the GUI files onto the server and point to the main html file */
app.use(serveStatic(demo_dir, {'index': ['index.html']}));

/* Handle incoming GET requests to run uname */
app.get("/run-uname", (request, response) => {
  execFile("/usr/bin/run_uname.sh", (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      response.status(500).send(stderr);
      return;
    }
    response.send(stdout);
  });
});

/* Open the server on port 8081 */
app.listen(8081, () => {
  console.log('Server is running on port 8081');
});