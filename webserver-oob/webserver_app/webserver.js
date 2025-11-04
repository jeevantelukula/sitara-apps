const express = require('express');
const { exec, spawn } = require('child_process');
const http = require('http');
const WebSocket = require('ws');
const pty = require('node-pty');
const fs = require('fs');

const app = express();
const server = http.createServer(app);
const port = 3000;

app.use(express.static('app'));

app.get('/run-uname', (req, res) => {
    exec('uname -a', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

app.get('/cpu-load', (req, res) => {
    exec('cpu_stats', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            return res.status(500).send(error);
        }
        res.send(stdout);
    });
});

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

server.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`);
});