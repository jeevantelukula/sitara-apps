/*
 * gc global variable provides access to GUI Composer infrastructure components and project information.
 * For more information, please see the Working with Javascript guide in the online help.
 */
var gc = gc || {};
gc.services = gc.services || {};

/*
 *  Boilerplate code for creating computed data bindings
 */
document.addEventListener('gc-databind-ready', function() {
    /*
     *   Add custom computed value databindings here, using the following method:
     *
     *   function gc.databind.registry.bind(targetBinding, modelBinding, [getter], [setter]);
     *
     *
     */
});

/*
 *  Boilerplate code for creating custom actions
 */
document.addEventListener('gc-nav-ready', function() {
    /*
     *   Add custom actions for menu items using the following api:
     *
     *   function gc.nav.registryAction(id, runable, [isAvailable], [isVisible]);
     *
     *
     */

    gc.nav.registerAction('open_log_pane', function() {
        if ((templateObj) && (templateObj.$)) {
            templateObj.$.ti_widget_eventlog_view.openView();
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });

    gc.nav.registerAction('open_scripting_window', function() {
        window.open('app/scripting.html', '_evm_scripting');
    }, function() {
        return true;
    }, function() {
        return true;
    });
});

/*
 *  Boilerplate code for working with components in the application gist
 */


var initComplete = false;
var templateObj;


// Wait for DOMContentLoaded event before trying to access the application template
var init = function() {
    console.log("init() function called.");
    templateObj = document.querySelector('#template_obj');
    console.log("templateObj after querySelector:", templateObj);

    // Wait for the template to fire a dom-change event to indicate that it has been 'stamped'
    // before trying to access components in the application.
    templateObj.addEventListener('dom-change', function() {
        if (initComplete) return;
        this.async(function() {
            initComplete = true;
            console.log("Application template has been stamped.");
            templateObj.$.ti_widget_toast.hideToast();
            templateObj.$.ti_widget_eventlog_view.log("info", "Application started.");

            // Expand vtabcontainer nav bar when user clicks on menu icon or 'Menu' label
            templateObj.toggleMenu = function(event){
                console.log("toggleMenu called. Current isExpanded:", templateObj.$.ti_widget_vtabcontainer.isExpanded);
                templateObj.$.ti_widget_vtabcontainer.isExpanded = !templateObj.$.ti_widget_vtabcontainer.isExpanded;
                console.log("New isExpanded:", templateObj.$.ti_widget_vtabcontainer.isExpanded);
            };
            templateObj.$.ti_widget_icon_button_menu.addEventListener('click',templateObj.toggleMenu);
            templateObj.$.ti_widget_label_menu.addEventListener('click',templateObj.toggleMenu);

            // Uname Demo specific logic
            const runUnameButton = templateObj.$.run_uname_button;
            const unameSysname = templateObj.$.uname_sysname;
            const unameNodename = templateObj.$.uname_nodename;
            const unameRelease = templateObj.$.uname_release;
            const unameVersion = templateObj.$.uname_version;
            const unameMachine = templateObj.$.uname_machine;
            const unameProcessor = templateObj.$.uname_processor;
            const unameOs = templateObj.$.uname_os;

            if (runUnameButton) {
                runUnameButton.addEventListener('click', function() {
                    // Set loading state
                    unameSysname.label = "Loading...";
                    unameNodename.label = "Loading...";
                    unameRelease.label = "Loading...";
                    unameVersion.label = "Loading...";
                    unameMachine.label = "Loading...";
                    unameProcessor.label = "Loading...";
                    unameOs.label = "Loading...";

                    $.get("/run-uname", function(data) {
                        const unameParts = data.trim().split(/\s+/);
                        if (unameParts.length >= 7) {
                            unameSysname.label = unameParts[0];
                            unameNodename.label = unameParts[1];
                            unameRelease.label = unameParts[2];
                            unameVersion.label = unameParts[3];
                            unameMachine.label = unameParts[4];
                            unameProcessor.label = unameParts[5];
                            unameOs.label = unameParts[6];
                        } else if (unameParts.length >= 6) {
                            // Handle cases where OS might be missing or combined
                            unameSysname.label = unameParts[0];
                            unameNodename.label = unameParts[1];
                            unameRelease.label = unameParts[2];
                            unameVersion.label = unameParts[3];
                            unameMachine.label = unameParts[4];
                            unameProcessor.label = unameParts[5];
                            unameOs.label = "N/A"; // Or try to infer
                        } else {
                            unameSysname.label = "Error: Invalid uname output";
                            unameNodename.label = "Error: Invalid uname output";
                            unameRelease.label = "Error: Invalid uname output";
                            unameVersion.label = "Error: Invalid uname output";
                            unameMachine.label = "Error: Invalid uname output";
                            unameProcessor.label = "Error: Invalid uname output";
                            unameOs.label = "Error: Invalid uname output";
                        }
                    }).fail(function(jqXHR, textStatus, errorThrown) {
                        const errorMessage = "Error fetching uname -a output: " + textStatus + " - " + errorThrown;
                        unameSysname.label = errorMessage;
                        unameNodename.label = errorMessage;
                        unameRelease.label = errorMessage;
                        unameVersion.label = errorMessage;
                        unameMachine.label = errorMessage;
                        unameProcessor.label = errorMessage;
                        unameOs.label = errorMessage;
                    });
                });
            }

            // Terminal specific logic
            const openTerminalButton = templateObj.$.open_terminal_button;
            const terminalVtab = templateObj.$.ti_widget_vtab_terminal;
            const terminalWidget = templateObj.$.terminal;

            let wsTerminal;

            if (openTerminalButton && terminalVtab && terminalWidget) {
                openTerminalButton.addEventListener('click', function() {
                    templateObj.$.ti_widget_vtabcontainer.selectedItem = terminalVtab;
                    
                    // Initialize terminal when tab is selected
                    if (!terminalWidget._initialized) {
                        console.log("Initializing terminal and connecting to WebSocket...");
                        
                        // Explicitly call _doInitialize with connectionMode 'None'
                        terminalWidget.connectionMode = "None";
                        terminalWidget._doInitialize();

                        wsTerminal = new WebSocket("ws://" + window.location.hostname + ":8082");

                        wsTerminal.onopen = function() {
                            console.log("Terminal WebSocket connected.");
                            terminalWidget.write("Connected to terminal WebSocket.\r\n");
                            
                            // Send initial terminal dimensions to the server
                            if (terminalWidget._term) {
                                const dimensions = { cols: terminalWidget._term.cols, rows: terminalWidget._term.rows };
                                wsTerminal.send(JSON.stringify({ type: "resize", ...dimensions }));
                            }
                        };

                        wsTerminal.onmessage = function(event) {
                            terminalWidget.write(event.data);
                        };

                        wsTerminal.onclose = function() {
                            console.log("Terminal WebSocket disconnected.");
                            terminalWidget.write("\r\nDisconnected from terminal WebSocket.\r\n");
                        };

                        wsTerminal.onerror = function(error) {
                            console.error("Terminal WebSocket error: ", error);
                            terminalWidget.write("\r\nWebSocket error: " + error.message + "\r\n");
                        };

                        // Handle input from the terminal and send it to the WebSocket server
                        terminalWidget.onKey(function(keyEvent) {
                            if (wsTerminal && wsTerminal.readyState === WebSocket.OPEN) {
                                wsTerminal.send(keyEvent.key);
                            }
                        });

                        terminalWidget._initialized = true;
                    }
                });
            }

            // Audio Classification Logic
            const audioClassificationVtab = templateObj.$.ti_widget_vtab_audio_classification;
            const startAudioButton = templateObj.$.start_audio_button;
            const stopAudioButton = templateObj.$.stop_audio_button;
            const audioClassificationResult = templateObj.$.audio_classification_result;
            const confidenceScore = templateObj.$.confidence_score;
            const loadingSpinner = document.getElementById('loading_spinner');
            const deviceListContainer = document.getElementById('device_list_container');

            let selectedDevice = null; // Will store the ALSA device ID (e.g., plughw:1,0)
            let selectedDeviceName = null; // Will store the display name
            let audioDevicesData = []; // Will store the full device list with names and IDs

            console.log("[Audio] Audio classification components initialized");

            // Fetch devices when the audio classification tab is selected
            templateObj.$.ti_widget_vtabcontainer.addEventListener('selected-item-changed', function(event) {
                if (event.detail.value === audioClassificationVtab.id) {
                    console.log("[Audio] Audio Classification tab selected, fetching devices...");
                    fetchAndDisplayAudioDevices();
                }
            });

            // Also fetch devices immediately if audio classification tab is already selected
            setTimeout(function() {
                if (templateObj.$.ti_widget_vtabcontainer.selectedItem?.id === audioClassificationVtab.id) {
                    console.log("[Audio] Audio Classification tab is already selected on page load");
                    fetchAndDisplayAudioDevices();
                }
            }, 500);

            // Function to fetch and display audio devices
            function fetchAndDisplayAudioDevices() {
                console.log("[Audio] fetchAndDisplayAudioDevices called");

                // Show loading state
                deviceListContainer.innerHTML = '<div class="loading-devices">Loading audio capture devices...</div>';
                startAudioButton.disabled = true;
                selectedDevice = null;
                selectedDeviceName = null;

                console.log("[Audio] Making GET request to /audio-devices");
                $.ajax({
                    url: "/audio-devices",
                    method: "GET",
                    dataType: "text",
                    success: function(data) {
                        console.log("[Audio] SUCCESS: Received response from /audio-devices");
                        console.log("[Audio] Raw data:", data);

                        const lines = data.trim().split('\n').filter(d => d.length > 0);
                        console.log("[Audio] Parsed lines:", lines);
                        console.log("[Audio] Number of lines:", lines.length);

                        // Check for error messages
                        if (lines.length === 0 ||
                            lines[0].toLowerCase().includes('error') ||
                            lines[0].toLowerCase().includes('no audio') ||
                            lines[0].toLowerCase().includes('not found')) {

                            console.warn("[Audio] No devices or error detected");
                            deviceListContainer.innerHTML = '<div class="no-devices-message">No audio capture devices found. Please connect a microphone.</div>';
                            audioClassificationResult.label = "No audio devices available";
                            return;
                        }

                        // Parse device names and prepare to fetch their ALSA IDs
                        audioDevicesData = lines.map(name => ({ name: name.trim(), alsaId: null }));
                        console.log("[Audio] Device names:", audioDevicesData.map(d => d.name));

                        // Now fetch the full device info with ALSA IDs
                        fetchDeviceDetails();
                    },
                    error: function(jqXHR, textStatus, errorThrown) {
                        console.error("[Audio] FAILED: Error fetching audio devices");
                        console.error("[Audio] Status:", textStatus);
                        console.error("[Audio] Error:", errorThrown);
                        console.error("[Audio] Response:", jqXHR.responseText);

                        deviceListContainer.innerHTML = '<div class="no-devices-message">Error loading devices. Check console for details.</div>';
                        audioClassificationResult.label = "Failed to load devices";
                    }
                });
            }

            // Function to fetch device details (ALSA IDs)
            function fetchDeviceDetails() {
                // The audio_utils binary outputs device names only
                // We need to map them to ALSA IDs (plughw:X,Y)
                // Based on audio_utils.c, devices are mapped to plughw:card,0
                // We'll need to parse this from the audio_utils or query the backend

                // For now, let's request the backend to provide full details
                // We'll modify the backend to return JSON with both name and ALSA ID
                displayDeviceList();
            }

            // Function to display device list with select buttons
            function displayDeviceList() {
                console.log("[Audio] Displaying device list");

                if (audioDevicesData.length === 0) {
                    deviceListContainer.innerHTML = '<div class="no-devices-message">No audio capture devices found.</div>';
                    return;
                }

                let html = '';
                audioDevicesData.forEach((device, index) => {
                    // Since audio_utils maps devices to plughw:card,0, we'll use the device name directly
                    // The backend will handle the mapping
                    html += `
                        <div class="device-item" data-device-index="${index}">
                            <div class="device-info">
                                <div class="device-name">${device.name}</div>
                                <div class="device-id">Audio Input Device ${index + 1}</div>
                            </div>
                            <button class="device-select-btn" data-device-index="${index}" data-device-name="${device.name}">
                                Select
                            </button>
                        </div>
                    `;
                });

                deviceListContainer.innerHTML = html;

                // Add click handlers to select buttons
                const selectButtons = deviceListContainer.querySelectorAll('.device-select-btn');
                selectButtons.forEach(btn => {
                    btn.addEventListener('click', function() {
                        const deviceIndex = parseInt(this.getAttribute('data-device-index'));
                        const deviceName = this.getAttribute('data-device-name');
                        selectAudioDevice(deviceIndex, deviceName);
                    });
                });

                audioClassificationResult.label = "Select a device above and click 'Start Classification'";
            }

            // Function to select a device
            function selectAudioDevice(index, deviceName) {
                console.log("[Audio] Device selected:", deviceName, "at index", index);

                selectedDevice = deviceName; // We'll pass the name to the backend
                selectedDeviceName = deviceName;

                // Update UI to show selection
                const allItems = deviceListContainer.querySelectorAll('.device-item');
                const allButtons = deviceListContainer.querySelectorAll('.device-select-btn');

                allItems.forEach(item => item.classList.remove('selected'));
                allButtons.forEach(btn => {
                    btn.classList.remove('selected');
                    btn.textContent = 'Select';
                });

                allItems[index].classList.add('selected');
                allButtons[index].classList.add('selected');
                allButtons[index].textContent = 'Selected ✓';

                // Enable start button
                startAudioButton.disabled = false;
                audioClassificationResult.label = `Ready to classify audio from: ${deviceName}`;

                console.log("[Audio] Device ready for classification:", selectedDevice);
            }

            // WebSocket for audio classification results
            let wsAudio = null;
            let isClassifying = false;

            // Function to set up WebSocket for audio classification results
            function setupAudioWebSocket() {
                // Close any existing WebSocket connection
                if (wsAudio) {
                    wsAudio.close();
                    wsAudio = null;
                }

                // Create new WebSocket connection
                wsAudio = new WebSocket("ws://" + window.location.hostname + ":" + window.location.port + "/audio");

                wsAudio.onopen = function() {
                    console.log("Audio Classification WebSocket connected.");
                };

                wsAudio.onmessage = function(event) {
                    try {
                        const result = JSON.parse(event.data);

                        // Handle different message types
                        if (result.status === 'connected') {
                            console.log("Audio WebSocket connected successfully");
                            // Keep existing UI state
                        } else if (result.status === 'stopped') {
                            console.log("Audio classification stopped via WebSocket");
                            updateUIState('stopped');
                        } else if (result.error) {
                            console.error("Error from WebSocket:", result.error);
                            audioClassificationResult.label = "Error: " + result.error;
                            confidenceScore.label = "Confidence: N/A";
                            updateUIState('error');
                        } else if (result.class) {
                            // Classification result
                            audioClassificationResult.label = "Class: " + result.class;
                            if (result.confidence !== undefined) {
                                const confidence = parseFloat(result.confidence);
                                const confidencePercent = isNaN(confidence) ? 0 : confidence * 100;
                                confidenceScore.label = "Confidence: " + confidencePercent.toFixed(1) + "%";
                            } else {
                                confidenceScore.label = "Confidence: N/A";
                            }
                            loadingSpinner.style.display = 'none';
                        }
                    } catch (e) {
                        console.error("Error parsing WebSocket message:", e);
                        // If not JSON, treat as plain text classification
                        const text = event.data.toString();
                        if (text && text.trim()) {
                            audioClassificationResult.label = "Class: " + text.trim();
                            confidenceScore.label = "Confidence: N/A";
                            loadingSpinner.style.display = 'none';
                        }
                    }
                };

                wsAudio.onclose = function() {
                    console.log("Audio Classification WebSocket disconnected.");
                    if (isClassifying) {
                        // Unexpected close while still classifying
                        audioClassificationResult.label = "Connection lost. Classification stopped.";
                        updateUIState('stopped');
                    }
                    wsAudio = null;
                };

                wsAudio.onerror = function(error) {
                    console.error("Audio Classification WebSocket error: ", error);
                    audioClassificationResult.label = "WebSocket error. Classification stopped.";
                    updateUIState('error');
                    if (wsAudio) {
                        wsAudio.close();
                        wsAudio = null;
                    }
                };
            }

            // Function to update UI state
            function updateUIState(state) {
                console.log("[Audio] UI state changing to:", state);
                switch(state) {
                    case 'starting':
                        isClassifying = true;
                        startAudioButton.disabled = true;
                        stopAudioButton.disabled = false;
                        // Disable device selection buttons
                        const selectButtons = deviceListContainer.querySelectorAll('.device-select-btn');
                        selectButtons.forEach(btn => btn.disabled = true);
                        audioClassificationResult.label = "Starting classification...";
                        confidenceScore.label = "Confidence: ";
                        loadingSpinner.style.display = 'block';
                        break;

                    case 'started':
                        isClassifying = true;
                        startAudioButton.disabled = true;
                        stopAudioButton.disabled = false;
                        audioClassificationResult.label = "Listening for audio...";
                        loadingSpinner.style.display = 'none';
                        break;

                    case 'stopping':
                        startAudioButton.disabled = true;
                        stopAudioButton.disabled = true;
                        audioClassificationResult.label = "Stopping classification...";
                        loadingSpinner.style.display = 'block';
                        break;

                    case 'stopped':
                        isClassifying = false;
                        startAudioButton.disabled = selectedDevice ? false : true; // Only enable if device is selected
                        stopAudioButton.disabled = true;
                        // Re-enable device selection buttons
                        const selectBtns = deviceListContainer.querySelectorAll('.device-select-btn');
                        selectBtns.forEach(btn => btn.disabled = false);
                        loadingSpinner.style.display = 'none';
                        if (audioClassificationResult.label === "Starting classification..." ||
                            audioClassificationResult.label === "Stopping classification...") {
                            audioClassificationResult.label = "Classification stopped. Select a device to start again.";
                        }
                        break;

                    case 'error':
                        isClassifying = false;
                        startAudioButton.disabled = selectedDevice ? false : true;
                        stopAudioButton.disabled = true;
                        // Re-enable device selection buttons
                        const selectBtnsError = deviceListContainer.querySelectorAll('.device-select-btn');
                        selectBtnsError.forEach(btn => btn.disabled = false);
                        loadingSpinner.style.display = 'none';
                        break;
                }
            }

            // Start audio classification
            startAudioButton.addEventListener('click', function() {
                if (!selectedDevice) {
                    audioClassificationResult.label = "Please select an audio device first";
                    return;
                }

                console.log("[Audio] Start button clicked, selected device:", selectedDevice);

                // Update UI to starting state
                updateUIState('starting');

                // Set up WebSocket first to ensure we catch early messages
                setupAudioWebSocket();

                // Send request to start classification with selected device
                console.log("[Audio] Sending start request with device:", selectedDevice);
                $.ajax({
                    url: "/start-audio-classification?device=" + encodeURIComponent(selectedDevice),
                    method: "GET",
                    dataType: "json",
                    timeout: 15000, // 15 seconds timeout
                    success: function(data) {
                        console.log("[Audio] Start classification response:", data);
                        if (data.status === 'started') {
                            updateUIState('started');
                            audioClassificationResult.label = `Classifying audio from: ${selectedDeviceName}`;
                            // Keep WebSocket open for results
                        } else {
                            audioClassificationResult.label = "Classification process completed.";
                            updateUIState('stopped');
                            if (wsAudio) {
                                wsAudio.close();
                                wsAudio = null;
                            }
                        }
                    },
                    error: function(jqXHR, textStatus, errorThrown) {
                        console.error("[Audio] Error starting audio classification:", jqXHR.responseText);
                        let errorMsg;
                        try {
                            const response = JSON.parse(jqXHR.responseText);
                            errorMsg = response.error || "Failed to start classification";
                        } catch (e) {
                            errorMsg = "Failed to start classification: " + textStatus;
                        }
                        audioClassificationResult.label = "Error: " + errorMsg;
                        updateUIState('error');
                        if (wsAudio) {
                            wsAudio.close();
                            wsAudio = null;
                        }
                    }
                });
            });

            // Stop audio classification
            stopAudioButton.addEventListener('click', function() {
                // Update UI to stopping state
                updateUIState('stopping');

                $.ajax({
                    url: "/stop-audio-classification",
                    method: "GET",
                    dataType: "json",
                    timeout: 10000,
                    success: function(data) {
                        console.log("Stop classification response:", data);
                        audioClassificationResult.label = "Classification stopped.";
                        updateUIState('stopped');
                        if (wsAudio) {
                            // Keep WebSocket open briefly to receive any final messages
                            setTimeout(function() {
                                if (wsAudio) {
                                    wsAudio.close();
                                    wsAudio = null;
                                }
                            }, 1000);
                        }
                    },
                    error: function(jqXHR, textStatus, errorThrown) {
                        console.error("Error stopping audio classification:", textStatus, errorThrown);
                        audioClassificationResult.label = "Error stopping classification. Please try again.";
                        updateUIState('error');
                    }
                });
            });

            function updateCpuLoad() {
                $.get("/cpu-load", function(data) {
                    templateObj.$.gauge1.value = data;
                });
            }

            setInterval(updateCpuLoad, 1000);
            updateCpuLoad();

        }, 1);

    });
};

templateObj = document.querySelector('#template_obj');
if (templateObj) {
    init();
} else {
    document.addEventListener('DOMContentLoaded', init);
}