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
    exec('/usr/bin/cpu_stats enhanced', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

/* Handle CPU info requests */
app.get('/cpu-info', (req, res) => {
    exec('/usr/bin/cpu_stats info', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

/* Audio classification support */
const fifoPath = '/tmp/audio_classification_fifo';

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

        // Send initial connected message
        ws.send(JSON.stringify({ status: 'connected', message: 'WebSocket connected, waiting for audio classification results' }));

        let fifoFd = null;
        let readBuffer = '';
        let readInterval = null;
        let isClassifying = true; // Assume active by default until told otherwise

        // Handle client messages
        ws.on('message', (message) => {
            try {
                const data = JSON.parse(message);
                console.log('[Audio WebSocket] Received client message:', data);

                if (data.type === 'client_status') {
                    if (data.status === 'stopped') {
                        console.log('[Audio WebSocket] Client stopped classification, pausing FIFO reading');
                        isClassifying = false;
                    } else if (data.status === 'started') {
                        console.log('[Audio WebSocket] Client started classification, resuming FIFO reading');
                        isClassifying = true;
                    }
                } else if (data.type === 'diagnostic_ping') {
                    console.log(`[Audio WebSocket] Received diagnostic ping #${data.counter}`);

                    // Send back diagnostic information
                    const diagnosticInfo = {
                        type: 'diagnostic_response',
                        fifo_exists: fs.existsSync(fifoPath),
                        fifo_fd: fifoFd !== null ? 'open' : 'closed',
                        buffer_size: readBuffer.length,
                        is_classifying: isClassifying,
                        timestamp: Date.now()
                    };

                    ws.send(JSON.stringify(diagnosticInfo));

                    // Force send a test message to verify WebSocket works
                    const testMessage = {
                        class: `TEST_CLASS_${data.counter}`,
                        // Remove confidence as requested
                        timestamp: Date.now()
                    };

                    ws.send(JSON.stringify(testMessage));
                }
            } catch (err) {
                console.error('[Audio WebSocket] Error parsing client message:', err.message);
            }
        });

        // Function to try opening and reading from FIFO
        const tryReadFifo = () => {
            try {
                // Skip reading if classification is paused
                if (!isClassifying) {
                    return; // Classification is paused by client
                }

                // Check if FIFO exists
                if (!fs.existsSync(fifoPath)) {
                    console.log('[Audio FIFO] FIFO does not exist yet, waiting for pipeline to create it');
                    return; // FIFO not created yet, wait for pipeline to start
                }

                // Try to open FIFO in non-blocking mode
                if (fifoFd === null) {
                    try {
                        console.log(`[Audio FIFO] Attempting to open FIFO at ${fifoPath}`);
                        fifoFd = fs.openSync(fifoPath, fs.constants.O_RDONLY | fs.constants.O_NONBLOCK);
                        console.log('[Audio FIFO] FIFO opened successfully for reading, fd:', fifoFd);
                    } catch (err) {
                        console.log('[Audio FIFO] Failed to open FIFO, it may not be ready yet:', err.message);
                        // FIFO not ready yet
                        return;
                    }
                }

                // Read available data
                const buffer = Buffer.alloc(4096);
                let bytesRead = 0;

                try {
                    bytesRead = fs.readSync(fifoFd, buffer, 0, buffer.length, null);
                    if (bytesRead > 0) {
                        console.log(`[Audio FIFO] Successfully read ${bytesRead} bytes from FIFO`);

                        // Log the first 100 characters of raw data for debugging
                        const rawData = buffer.toString('utf8', 0, Math.min(bytesRead, 100));
                        console.log(`[Audio FIFO] Raw data sample: "${rawData}${bytesRead > 100 ? '...' : ''}"`);

                        // Check for dollar sign delimiters
                        if (buffer.toString('utf8', 0, bytesRead).indexOf('$') === -1) {
                            console.log('[Audio FIFO] Warning: No dollar sign delimiters found in data');
                        }
                    }
                } catch (err) {
                    if (err.code === 'EAGAIN' || err.code === 'EWOULDBLOCK') {
                        // No data available yet
                        return;
                    }
                    console.error('[Audio FIFO] Error reading from FIFO:', err.message);
                    throw err;
                }

                if (bytesRead > 0) {
                    // Append to read buffer
                    readBuffer += buffer.toString('utf8', 0, bytesRead);
                    console.log('[Audio FIFO] Buffer now contains:', readBuffer.length, 'bytes');

                    // Process complete results (try multiple possible delimiters)
                    let resultCount = 0;
                    let result = null;
                    let delimiterFound = false;

                    // Try to extract using regex pattern matching for audio class names
                    // This is a more robust approach that can extract words even from messy output
                    const classRegex = /\b([A-Za-z_]+)\b/g;
                    const matches = readBuffer.match(classRegex);
                    if (matches && matches.length > 0) {
                        // Just use the first word we find as the classification
                        delimiterFound = true;
                        result = matches[0].trim();
                        console.log(`[Audio FIFO] Found result using regex: "${result}"`);

                        // Remove everything up to and including this match from the buffer
                        const matchIndex = readBuffer.indexOf(result);
                        if (matchIndex !== -1) {
                            readBuffer = readBuffer.substring(matchIndex + result.length);
                            resultCount++;
                        }
                    }

                    // If no regex match, try dollar sign delimiter as a fallback
                    if (!delimiterFound) {
                        let dollarIndex = readBuffer.indexOf('$');
                        if (dollarIndex !== -1) {
                            delimiterFound = true;
                            result = readBuffer.substring(0, dollarIndex).trim();
                            readBuffer = readBuffer.substring(dollarIndex + 1);
                            resultCount++;
                            console.log(`[Audio FIFO] Found result using $ delimiter: "${result}"`);
                        }
                    }

                    // If no dollar sign, try newline delimiter
                    if (!delimiterFound) {
                        let newlineIndex = readBuffer.indexOf('\n');
                        if (newlineIndex !== -1) {
                            delimiterFound = true;
                            result = readBuffer.substring(0, newlineIndex).trim();
                            readBuffer = readBuffer.substring(newlineIndex + 1);
                            resultCount++;
                            console.log(`[Audio FIFO] Found result using newline delimiter: "${result}"`);
                        }
                    }

                    // If nothing found yet but buffer is getting large, try to extract anything useful
                    if (!delimiterFound && readBuffer.length > 1000) {
                        console.log(`[Audio FIFO] No delimiter found but buffer large (${readBuffer.length} bytes), using whole buffer`);
                        result = readBuffer.trim();
                        readBuffer = '';
                        resultCount++;

                        // Try to find and extract just a word if it's a messy buffer
                        const wordMatch = result.match(/[A-Za-z_]+/);
                        if (wordMatch) {
                            console.log(`[Audio FIFO] Extracted word from buffer: "${wordMatch[0]}"`);
                            result = wordMatch[0];
                        }
                    }

                    // Process the result we found (if any)
                    if (result !== null) {
                        console.log(`[Audio FIFO] Extracted result: "${result}"`);

                        if (result && ws.readyState === WebSocket.OPEN) {
                            console.log(`[Audio FIFO] Sending classification result: "${result}"`);

                            // Send as JSON
                            const resultData = {
                                class: result,
                                confidence: 1.0,
                                timestamp: Date.now()
                            };

                            try {
                                ws.send(JSON.stringify(resultData));
                                console.log('[Audio FIFO] Sent result to WebSocket client');
                            } catch (wsErr) {
                                console.error('[Audio FIFO] Error sending to WebSocket:', wsErr.message);
                            }
                        } else {
                            // Log why we're not sending
                            if (!result) {
                                console.log('[Audio FIFO] Not sending: empty result');
                            } else if (!ws || ws.readyState !== WebSocket.OPEN) {
                                console.log(`[Audio FIFO] Not sending: WebSocket not ready, readyState=${ws ? ws.readyState : 'ws is null'}`);
                            }
                        }
                    }

                    if (resultCount > 0) {
                        console.log(`[Audio FIFO] Processed ${resultCount} classification results`);
                    } else {
                        console.log('[Audio FIFO] No complete results found in buffer yet');
                    }
                }
            } catch (err) {
                console.error(`[Audio FIFO] Error reading from FIFO: ${err.message}`);
                console.error(`[Audio FIFO] Error stack: ${err.stack}`);
                if (fifoFd !== null) {
                    try {
                        fs.closeSync(fifoFd);
                        console.log('[Audio FIFO] Closed FIFO file descriptor after error');
                    } catch (e) {
                        console.error('[Audio FIFO] Error closing FIFO:', e.message);
                    }
                    fifoFd = null;
                }
            }
        };

        // Poll FIFO every 500ms (reduced frequency for more reliable updates)
        console.log('[Audio FIFO] Setting up polling interval (500ms)');
        readInterval = setInterval(tryReadFifo, 500);

        ws.on('close', () => {
            console.log('Audio WebSocket connection closed');
            if (readInterval) {
                clearInterval(readInterval);
                readInterval = null;
            }
            if (fifoFd !== null) {
                try {
                    fs.closeSync(fifoFd);
                } catch (err) {}
                fifoFd = null;
            }
        });

        ws.on('error', (err) => {
            console.error(`Audio WebSocket error: ${err.message}`);
            if (readInterval) {
                clearInterval(readInterval);
                readInterval = null;
            }
            if (fifoFd !== null) {
                try {
                    fs.closeSync(fifoFd);
                } catch (e) {}
                fifoFd = null;
            }
        });
    } else {
        console.log(`Unknown WebSocket URL: ${req.url}`);
        ws.close(1003, "Unsupported WebSocket URL");
    }
});

/* Audio classification support */
let audioProcess = null;

// Helper function to force kill any GStreamer processes
function forceKillGstreamer() {
    console.log('[Force Cleanup] Looking for stray GStreamer processes');
    try {
        // Use execSync to get immediate results
        const { execSync } = require('child_process');
        try {
            // Find GStreamer processes
            const psOutput = execSync('ps aux | grep gst-launch | grep -v grep', { encoding: 'utf8' });

            if (psOutput && psOutput.trim()) {
                console.log('[Force Cleanup] Found GStreamer processes:', psOutput);
                // Extract PIDs
                const lines = psOutput.trim().split('\n');
                for (const line of lines) {
                    const parts = line.trim().split(/\s+/);
                    if (parts.length > 1) {
                        const pid = parts[1];
                        console.log('[Force Cleanup] Killing PID:', pid);
                        try {
                            // Force kill with SIGKILL
                            execSync(`kill -9 ${pid}`);
                            console.log('[Force Cleanup] Successfully killed PID:', pid);
                        } catch (killError) {
                            console.error('[Force Cleanup] Failed to kill PID:', pid, killError.message);
                        }
                    }
                }
            } else {
                console.log('[Force Cleanup] No GStreamer processes found');
            }
        } catch (psError) {
            // If grep returns no results, it will exit with code 1
            if (psError.status === 1) {
                console.log('[Force Cleanup] No GStreamer processes found');
            } else {
                console.error('[Force Cleanup] Error checking for GStreamer processes:', psError.message);
            }
        }

        // Clean up any FIFO or PID files
        if (fs.existsSync('/tmp/audio_classification.pid')) {
            fs.unlinkSync('/tmp/audio_classification.pid');
            console.log('[Force Cleanup] Removed stale PID file');
        }

        if (fs.existsSync(fifoPath)) {
            fs.unlinkSync(fifoPath);
            console.log('[Force Cleanup] Removed stale FIFO');
        }
    } catch (err) {
        console.error('[Force Cleanup] Error during cleanup:', err.message);
    }
}

app.get('/audio-devices', (req, res) => {
    console.log('[/audio-devices] Endpoint called');

    // Use standard /usr/bin location for binary
    const audioUtilsPath = '/usr/bin/audio_utils';

    console.log(`[/audio-devices] Executing: ${audioUtilsPath} devices`);
    exec(`${audioUtilsPath} devices`, (error, stdout, stderr) => {
        if (error) {
            console.error(`[/audio-devices] Failed to get audio devices: ${error}`);
            console.error(`[/audio-devices] stderr: ${stderr}`);
            console.error(`[/audio-devices] error code: ${error.code}`);
            console.error(`[/audio-devices] Attempted path: ${audioUtilsPath}`);
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

    console.log(`[/start-audio-classification] Starting audio classification with device: ${device}`);

    // Use standard /usr/bin location for binary
    const audioUtilsPath = '/usr/bin/audio_utils';

    console.log(`[/start-audio-classification] Executing: ${audioUtilsPath} start_gst ${device}`);

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
    console.log('[/stop-audio-classification] Stopping audio classification');

    // Start with force cleanup to ensure any stray processes are killed
    forceKillGstreamer();

    if (audioProcess) {
        console.log('[/stop-audio-classification] Stopping managed audio process');

        // Use standard /usr/bin location for binary
        const audioUtilsPath = '/usr/bin/audio_utils';

        console.log(`[/stop-audio-classification] Executing: ${audioUtilsPath} stop_gst`);

        // Send the stop command
        const stopProcess = spawn(audioUtilsPath, ['stop_gst']);

        // Set a timeout for graceful shutdown
        const forceKillTimeout = setTimeout(() => {
            console.log('[/stop-audio-classification] Timeout waiting for clean stop, force killing');

            // Kill the audio process if it still exists
            if (audioProcess) {
                console.log('[/stop-audio-classification] Killing audio process with SIGKILL');
                try {
                    audioProcess.kill('SIGKILL');
                } catch (e) {
                    console.error('[/stop-audio-classification] Error killing process:', e.message);
                }
                audioProcess = null;
            }

            // Run force cleanup again
            forceKillGstreamer();
        }, 3000); // 3 second timeout for graceful shutdown

        stopProcess.stdout.on('data', (data) => {
            console.log(`[/stop-audio-classification] Stop stdout: ${data.toString().trim()}`);
        });

        stopProcess.stderr.on('data', (data) => {
            console.log(`[/stop-audio-classification] Stop stderr: ${data.toString().trim()}`);
        });

        stopProcess.on('close', (code) => {
            console.log(`[/stop-audio-classification] Stop command exited with code ${code}`);
            clearTimeout(forceKillTimeout);

            // Ensure the main process is killed
            setTimeout(() => {
                if (audioProcess) {
                    console.log('[/stop-audio-classification] Killing audio process');
                    try {
                        audioProcess.kill();
                    } catch (e) {
                        console.error('[/stop-audio-classification] Error killing process:', e.message);
                    }
                    audioProcess = null;
                }

                // Run force cleanup one more time to be sure
                forceKillGstreamer();

                res.status(200).json({ status: 'stopped', message: 'Audio classification stopped' });
            }, 500);
        });

        stopProcess.on('error', (err) => {
            console.error(`[/stop-audio-classification] Failed to stop audio classification: ${err.message}`);
            clearTimeout(forceKillTimeout);

            // Fallback to killing the process directly
            if (audioProcess) {
                try {
                    audioProcess.kill('SIGKILL');
                } catch (e) {
                    console.error('[/stop-audio-classification] Error killing process:', e.message);
                }
                audioProcess = null;
            }

            // Run force cleanup again
            forceKillGstreamer();

            res.status(200).json({ status: 'stopped', message: 'Audio classification stopped (fallback)' });
        });
    } else {
        // No managed process, but run cleanup anyway
        console.log('[/stop-audio-classification] No managed audio process, but running cleanup');

        // Run cleanup one more time
        forceKillGstreamer();

        res.status(200).json({ status: 'not_running', message: 'Audio classification not running' });
    }
});

/* Start the server */
// Run cleanup on startup to ensure no stale processes
console.log('[STARTUP] Running initial cleanup to ensure no stale processes');
forceKillGstreamer();

server.listen(port, () => {
    console.log(`Webserver-OOB listening on port ${port}, serving from ${app_dir}`);
});
