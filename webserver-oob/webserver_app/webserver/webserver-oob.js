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
    console.log(`New WebSocket connection: ${req.url}`);

    if (req.url === '/terminal') {
        const shell = pty.spawn('/bin/bash', [], {
            name: 'xterm-color',
            cols: 80,
            rows: 30,
            cwd: process.env.HOME,
            env: process.env
        });

        shell.on('data', (data) => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(data);
            }
        });

        ws.on('message', (message) => {
            shell.write(message);
        });

        ws.on('close', () => {
            console.log('Terminal WebSocket connection closed');
            shell.kill();
        });

        ws.on('error', (err) => {
            console.error(`Terminal WebSocket error: ${err.message}`);
            shell.kill();
        });
    } else if (req.url === '/audio') {
        console.log(`Setting up audio classification WebSocket on ${fifoPath}`);

        // Check if FIFO exists, if not create it
        if (!fs.existsSync(fifoPath)) {
            try {
                require('child_process').execSync(`mkfifo ${fifoPath}`);
                console.log(`Created FIFO at ${fifoPath}`);
            } catch (err) {
                console.error(`Failed to create FIFO: ${err.message}`);
                ws.close(1011, "Failed to create FIFO");
                return;
            }
        }

        let fifoStream;

        try {
            // Open the FIFO for reading
            fifoStream = fs.createReadStream(fifoPath, { flags: 'r' });

            fifoStream.on('data', (data) => {
                try {
                    const result = data.toString().trim();
                    console.log(`Audio classification result: ${result}`);

                    if (ws.readyState === WebSocket.OPEN) {
                        // Format the result as JSON
                        let resultData;
                        try {
                            // Try to parse the result in case it's already JSON
                            resultData = JSON.parse(result);
                        } catch (e) {
                            // If not, create a simple JSON object
                            resultData = { class: result, confidence: 1.0 };
                        }

                        ws.send(JSON.stringify(resultData));
                    }
                } catch (err) {
                    console.error(`Error processing audio classification data: ${err.message}`);
                }
            });

            fifoStream.on('error', (err) => {
                console.error(`FIFO stream error: ${err.message}`);
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({ error: 'FIFO stream error', message: err.message }));
                }
            });

            fifoStream.on('close', () => {
                console.log('FIFO stream closed');
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({ status: 'stopped', message: 'Audio classification stopped' }));
                }
            });

            ws.on('close', () => {
                console.log('Audio WebSocket connection closed');
                if (fifoStream) {
                    fifoStream.close();
                }
            });

            ws.on('error', (err) => {
                console.error(`Audio WebSocket error: ${err.message}`);
                if (fifoStream) {
                    fifoStream.close();
                }
            });

            // Send initial connected message
            ws.send(JSON.stringify({ status: 'connected', message: 'WebSocket connected, waiting for audio classification results' }));

        } catch (err) {
            console.error(`Error setting up FIFO stream: ${err.message}`);
            ws.close(1011, "Failed to set up FIFO stream");
        }
    } else {
        console.log(`Unknown WebSocket URL: ${req.url}`);
        ws.close(1003, "Unsupported WebSocket URL");
    }
});

/* Audio classification support */
let audioProcess = null;
const fifoPath = '/tmp/audio_classification_fifo';

app.get('/audio-devices', (req, res) => {
    console.log('[/audio-devices] Endpoint called');

    // Use full path to audio_utils binary
    const audioUtilsPath = '/usr/share/webserver-oob/webserver_app/linux_app/audio_utils';

    console.log(`[/audio-devices] Executing: ${audioUtilsPath} devices`);
    exec(`${audioUtilsPath} devices`, (error, stdout, stderr) => {
        if (error) {
            console.error(`[/audio-devices] Failed to get audio devices: ${error}`);
            console.error(`[/audio-devices] stderr: ${stderr}`);
            // Send plain text error message, not JSON
            return res.status(500).send(`Error: ${error.message}\n${stderr}`);
        }

        // Log both stdout and stderr for debugging
        console.log(`[/audio-devices] Audio devices stdout: ${stdout.trim()}`);
        if (stderr) {
            console.log(`[/audio-devices] Audio devices stderr: ${stderr.trim()}`);
        }

        // Return the list of devices from stdout (plain text)
        console.log(`[/audio-devices] Sending response with ${stdout.trim().split('\n').length} lines`);
        res.send(stdout);
    });
});

app.get('/start-audio-classification', (req, res) => {
    const device = req.query.device;
    if (!device) {
        return res.status(400).json({ error: 'Missing device parameter' });
    }

    if (audioProcess) {
        return res.status(400).json({ error: 'Audio classification already running' });
    }

    console.log(`Starting audio classification with device: ${device}`);

    // Use full path to audio_utils binary
    const audioUtilsPath = '/usr/share/webserver-oob/webserver_app/linux_app/audio_utils';

    // Start the audio_utils process
    audioProcess = spawn(audioUtilsPath, ['start_gst', device]);

    // Set up a timeout to check if the process started successfully
    const startTimeout = setTimeout(() => {
        if (audioProcess) {
            console.error('Audio classification start timeout - no response received');
            res.status(500).json({ error: 'Audio classification start timeout' });
            audioProcess.kill();
            audioProcess = null;
        }
    }, 5000);

    let outputBuffer = '';

    audioProcess.stdout.on('data', (data) => {
        const output = data.toString().trim();
        console.log(`audio_utils stdout: ${output}`);

        outputBuffer += output;

        // Check for success/error messages
        if (output.includes('SUCCESS:')) {
            clearTimeout(startTimeout);
            res.status(200).json({ status: 'started', message: 'Audio classification started successfully' });
        } else if (output.includes('ERROR:')) {
            clearTimeout(startTimeout);
            res.status(500).json({ error: output.replace('ERROR:', '').trim() });
            audioProcess.kill();
            audioProcess = null;
        }
    });

    audioProcess.stderr.on('data', (data) => {
        console.error(`audio_utils stderr: ${data}`);
    });

    audioProcess.on('error', (err) => {
        clearTimeout(startTimeout);
        console.error(`Failed to start audio_utils: ${err}`);
        res.status(500).json({ error: `Failed to start audio classification: ${err.message}` });
        audioProcess = null;
    });

    audioProcess.on('close', (code) => {
        clearTimeout(startTimeout);
        console.log(`audio_utils process exited with code ${code}`);

        // Only send response if we haven't already
        if (!res.headersSent) {
            if (code === 0) {
                res.status(200).json({ status: 'completed', message: 'Audio classification completed' });
            } else {
                res.status(500).json({ error: `Audio classification failed with code ${code}` });
            }
        }
        audioProcess = null;
    });
});

app.get('/stop-audio-classification', (req, res) => {
    if (audioProcess) {
        console.log('Stopping audio classification');

        // Use full path to audio_utils binary
        const audioUtilsPath = '/usr/share/webserver-oob/webserver_app/linux_app/audio_utils';

        // Send the stop command instead of killing the process
        const stopProcess = spawn(audioUtilsPath, ['stop_gst']);

        stopProcess.on('close', (code) => {
            console.log(`Stop command exited with code ${code}`);

            // Ensure the main process is killed after a short delay
            setTimeout(() => {
                if (audioProcess) {
                    audioProcess.kill();
                    audioProcess = null;
                }
            }, 500);

            res.status(200).json({ status: 'stopped', message: 'Audio classification stopped' });
        });

        stopProcess.on('error', (err) => {
            console.error(`Failed to stop audio classification: ${err}`);

            // Fallback to killing the process directly
            audioProcess.kill();
            audioProcess = null;
            res.status(200).json({ status: 'stopped', message: 'Audio classification stopped (fallback)' });
        });
    } else {
        res.status(200).json({ status: 'not_running', message: 'Audio classification not running' });
    }
});

/* Start the server */
server.listen(port, () => {
    console.log(`Webserver-OOB listening on port ${port}, serving from ${app_dir}`);
});
