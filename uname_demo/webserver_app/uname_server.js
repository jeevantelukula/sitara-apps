const express = require('express');
const { execFile } = require('child_process');
const path = require('path');

const app = express();
const port = 8081; // Use port 8081 as requested

// Serve static files from the 'app' directory
app.use('/app', express.static(path.join(__dirname, 'app')));

// Endpoint to execute the cpu_stats binary
app.get('/cgi-bin/cpu_stats', (req, res) => {
    const cpuStatsPath = path.join(__dirname, 'linux_app', 'cpu_stats');

    execFile(cpuStatsPath, (error, stdout, stderr) => {
        if (error) {
            console.error(`execFile error: ${error}`);
            res.status(500).send(`Error executing cpu_stats: ${stderr}`);
            return;
        }
        
        const parts = stdout.split('\n\n');
        const body = parts.length > 1 ? parts[1] : parts[0];

        res.setHeader('Content-Type', 'application/json');
        res.send(body);
    });
});

// Endpoint to execute lscpu and return parsed JSON
app.get('/api/lscpu', (req, res) => {
    execFile('lscpu', (error, stdout, stderr) => {
        if (error) {
            console.error(`lscpu error: ${error}`);
            res.status(500).send(`Error executing lscpu: ${stderr}`);
            return;
        }

        const lines = stdout.split('\n');
        const cpuInfo = {};
        const relevantKeys = ['Architecture', 'Vendor ID', 'Model name'];

        lines.forEach(line => {
            const parts = line.split(/:\s+/);
            if (parts.length === 2) {
                const key = parts[0].trim();
                const value = parts[1].trim();
                if (relevantKeys.includes(key)) {
                    cpuInfo[key] = value;
                }
            }
        });

        res.setHeader('Content-Type', 'application/json');
        res.json(cpuInfo);
    });
});

// Redirect root to the main app page
app.get('/', (req, res) => {
    res.redirect('/app/index.html');
});

app.listen(port, () => {
    console.log(`Uname Demo server running at http://localhost:${port}`);
});
