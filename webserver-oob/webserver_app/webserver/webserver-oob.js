#!/usr/bin/env node

/*
 * Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Libraries
 */
const express = require('express');
const { exec, spawn } = require('child_process');
const http = require('http');
const WebSocket = require('ws');
const pty = require('node-pty');
const fs = require('fs');

/*
 * Definitions
 */

/* Set the path from command line argument */
const app_dir = process.argv[2] || 'app';

/* Create the express app */
const app = express();
const server = http.createServer(app);
const port = 3000;

/*
 * Server Code
 */

/* Place the GUI files onto the server and point to the main html file */
app.use(express.static(app_dir));

/* Handle system information requests */
app.get('/run-uname', (req, res) => {
    exec('uname -a', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

/* Handle CPU load requests */
app.get('/cpu-load', (req, res) => {
    exec('/usr/share/webserver-oob/webserver_app/linux_app/cpu_stats enhanced', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

/* Handle CPU info requests */
app.get('/cpu-info', (req, res) => {
    exec('/usr/share/webserver-oob/webserver_app/linux_app/cpu_stats info', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

/* WebSocket server for terminal and audio */
const wss = new WebSocket.Server({ server });

wss.on('connection', (ws, req) => {
    if (req.url === '/terminal') {
        const shell = pty.spawn('/bin/bash', [], {
            name: 'xterm-color',
            cols: 80,
            rows: 30,
            cwd: process.env.HOME,
            env: process.env
        });

        shell.on('data', (data) => {
            ws.send(data);
        });

        ws.on('message', (message) => {
            shell.write(message);
        });

        ws.on('close', () => {
            shell.kill();
        });
    } else if (req.url === '/audio') {
        const fifoStream = fs.createReadStream(fifoPath);

        fifoStream.on('data', (data) => {
            ws.send(data.toString());
        });

        ws.on('close', () => {
            fifoStream.close();
        });
    }
});

/* Audio classification support */
let audioProcess = null;
const fifoPath = '/tmp/audio_classification_fifo';

app.get('/audio-devices', (req, res) => {
    exec('audio_utils devices', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

app.get('/start-audio-classification', (req, res) => {
    const device = req.query.device;
    if (!device) {
        return res.status(400).send('Missing device parameter');
    }

    if (audioProcess) {
        return res.status(400).send('Audio classification already running');
    }

    audioProcess = spawn('audio_utils', ['start_gst', device]);

    audioProcess.stdout.on('data', (data) => {
        console.log(`audio_utils stdout: ${data}`);
    });

    audioProcess.stderr.on('data', (data) => {
        console.error(`audio_utils stderr: ${data}`);
    });

    audioProcess.on('close', (code) => {
        console.log(`audio_utils process exited with code ${code}`);
        audioProcess = null;
    });

    res.send('Audio classification started');
});

app.get('/stop-audio-classification', (req, res) => {
    if (audioProcess) {
        audioProcess.kill();
        audioProcess = null;
        res.send('Audio classification stopped');
    } else {
        res.status(400).send('Audio classification not running');
    }
});

/* Start the server */
server.listen(port, () => {
    console.log(`Webserver-OOB listening on port ${port}, serving from ${app_dir}`);
});
