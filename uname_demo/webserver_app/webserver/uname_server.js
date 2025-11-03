/*
 * Libraries
 */
const express = require('express');
const { execFile, spawn } = require('child_process');
const serveStatic = require('serve-static');
const path = require('path');
const http = require('http');
const WebSocket = require('ws');
const pty = require('node-pty');
const fs = require('fs');

/*
 * Definitions
 */

/* Set the path */
const demo_dir = process.env.APP_DIR || path.join(__dirname, '../../webserver_app/app');
const linux_app_dir = path.join(__dirname, '../../webserver_app/linux_app');

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
  const scriptPath = path.join(linux_app_dir, 'run_uname.sh');
  execFile(scriptPath, (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      response.status(500).send(stderr);
      return;
    }
    response.send(stdout);
  });
});

/* Handle incoming GET requests for audio devices */
app.get("/audio-devices", (request, response) => {
  const audioUtilsPath = path.join(linux_app_dir, 'audio_utils');
  execFile(audioUtilsPath, ['devices'], (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      response.status(500).send(stderr);
      return;
    }
    response.send(stdout);
  });
});

let gstProcess = null;

/* Handle incoming GET requests to start audio classification */
app.get("/start-audio-classification", (request, response) => {
  const audioUtilsPath = path.join(linux_app_dir, 'audio_utils');
  const device = request.query.device || "plughw:1,0";

  if (gstProcess) {
    response.status(400).send("GStreamer process already running.");
    return;
  }

  gstProcess = spawn(audioUtilsPath, ['start_gst', device]);

  gstProcess.stdout.on('data', (data) => {
    console.log(`GStreamer stdout: ${data}`);
  });

  gstProcess.stderr.on('data', (data) => {
    console.error(`GStreamer stderr: ${data}`);
  });

  gstProcess.on('close', (code) => {
    console.log(`GStreamer process exited with code ${code}`);
    gstProcess = null;
  });

  response.send("GStreamer process started.");
});

/* Handle incoming GET requests to stop audio classification */
app.get("/stop-audio-classification", (request, response) => {
  const audioUtilsPath = path.join(linux_app_dir, 'audio_utils');

  if (!gstProcess) {
    response.status(400).send("GStreamer process not running.");
    return;
  }

  execFile(audioUtilsPath, ['stop_gst'], (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      response.status(500).send(stderr);
      return;
    }
    response.send("GStreamer process stopped.");
  });
});

/* Create a websocket server for the terminal */
const wss = new WebSocket.Server({ port: 8082 });

wss.on('connection', (ws) => {
  let term;

  ws.on('message', (message) => {
    try {
      const parsedMessage = JSON.parse(message);
      if (parsedMessage.type === "resize" && term) {
        term.resize(parsedMessage.cols, parsedMessage.rows);
      } else {
        if (term) {
          term.write(message);
        }
      }
    } catch (e) {
      // If not JSON, treat as regular terminal input
      if (term) {
        term.write(message);
      }
    }
  });

  // Initialize pty after first message (which should be resize)
  ws.once('message', (message) => {
    try {
      const parsedMessage = JSON.parse(message);
      if (parsedMessage.type === "resize") {
        term = pty.spawn('/bin/bash', [], {
          name: 'xterm-color',
          cols: parsedMessage.cols,
          rows: parsedMessage.rows,
          cwd: process.env.HOME,
          env: process.env
        });

        term.on('data', (data) => {
          ws.send(data);
        });

        term.on('close', () => {
          ws.close();
        });

        ws.on('close', () => {
          term.kill();
        });
      } else {
        // If first message is not resize, spawn with default size
        term = pty.spawn('/bin/bash', [], {
          name: 'xterm-color',
          cols: 80,
          rows: 30,
          cwd: process.env.HOME,
          env: process.env
        });
        term.on('data', (data) => {
          ws.send(data);
        });
        term.on('close', () => {
          ws.close();
        });
        ws.on('close', () => {
          term.kill();
        });
        // Process the initial message as regular input
        term.write(message);
      }
    } catch (e) {
      // If first message is not JSON, spawn with default size
      term = pty.spawn('/bin/bash', [], {
        name: 'xterm-color',
        cols: 80,
        rows: 30,
        cwd: process.env.HOME,
        env: process.env
      });
      term.on('data', (data) => {
        ws.send(data);
      });
      term.on('close', () => {
        ws.close();
      });
      ws.on('close', () => {
        term.kill();
      });
      // Process the initial message as regular input
      term.write(message);
    }
  });
});

// WebSocket server for audio classification results
const audioWsServer = new WebSocket.Server({ port: 8083 });
const fifoPath = "/tmp/audio_classification_fifo";

audioWsServer.on('connection', (ws) => {
  console.log('Audio Classification WebSocket client connected.');

  let fifoFd;
  let readStream;

  const openFifo = () => {
    if (fs.existsSync(fifoPath)) {
      fifoFd = fs.openSync(fifoPath, 'r');
      readStream = fs.createReadStream(null, { fd: fifoFd });

      readStream.on('data', (data) => {
        ws.send(data.toString().trim());
      });

      readStream.on('end', () => {
        console.log('FIFO read stream ended.');
        if (fifoFd) fs.closeSync(fifoFd);
        fifoFd = null;
        readStream = null;
        // Re-open FIFO if it's still expected to be active
        if (gstProcess) {
          setTimeout(openFifo, 1000); // Try to re-open after a delay
        }
      });

      readStream.on('error', (err) => {
        console.error('FIFO read stream error:', err);
        if (fifoFd) fs.closeSync(fifoFd);
        fifoFd = null;
        readStream = null;
        if (gstProcess) {
          setTimeout(openFifo, 1000); // Try to re-open after a delay
        }
      });
    } else {
      console.log('FIFO does not exist yet, waiting...');
      if (gstProcess) {
        setTimeout(openFifo, 1000); // Try again after a delay
      }
    }
  };

  openFifo();

  ws.on('close', () => {
    console.log('Audio Classification WebSocket client disconnected.');
    if (readStream) {
      readStream.destroy();
      readStream = null;
    }
    if (fifoFd) {
      fs.closeSync(fifoFd);
      fifoFd = null;
    }
  });

  ws.on('error', (err) => {
    console.error('Audio Classification WebSocket error:', err);
  });
});

/* Open the server on port 8081 */
server.listen(8081, () => {
  console.log('Server is running on http://localhost:8081');
});