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

            const audioClassificationVtab = templateObj.$.ti_widget_vtab_audio_classification;
            const audioDeviceDroplist = templateObj.$.audio_device_droplist;
            const startAudioButton = templateObj.$.start_audio_button;
            const stopAudioButton = templateObj.$.stop_audio_button;
            const audioClassificationResult = templateObj.$.audio_classification_result;
            const confidenceScore = templateObj.$.confidence_score;
            const loadingSpinner = templateObj.$.loading_spinner;

            let wsAudio;

            // Fetch devices when the audio classification tab is selected
            templateObj.$.ti_widget_vtabcontainer.addEventListener('selected-item-changed', function(event) {
                console.log("selected-item-changed event fired. Detail value:", event.detail.value, "Audio Classification Vtab ID:", audioClassificationVtab.id);
                if (event.detail.value === audioClassificationVtab.id) {
                    fetchAudioDevices();
                }
            });

            // Function to fetch audio devices
            const fetchAudioDevices = function() {
                // Show loading indicator in dropdown
                audioDeviceDroplist.labels = "Loading devices...";
                audioDeviceDroplist.selectedIndex = -1;

                $.get("/audio-devices", function(data) {
                    console.log("Raw data from /audio-devices:", data);
                    const devices = data.trim().split('\n').filter(d => d.length > 0);
                    console.log("Parsed devices array:", devices);

                    if (devices.length > 0) {
                        // Check if the first entry contains an error message
                        if (devices[0].toLowerCase().includes('error') ||
                            devices[0].toLowerCase().includes('no audio') ||
                            devices[0].toLowerCase().includes('not found')) {

                            // Error message in the response
                            console.warn("Device error detected:", devices[0]);
                            audioDeviceDroplist.labels = devices[0];
                            audioDeviceDroplist.selectedIndex = -1;

                            // Disable start button
                            startAudioButton.disabled = true;

                            // Update status message
                            audioClassificationResult.label = "No audio devices available";
                            confidenceScore.label = "Confidence: N/A";
                        } else {
                            // Valid devices found
                            audioDeviceDroplist.labels = devices.join('|');
                            console.log("Setting droplist labels to:", audioDeviceDroplist.labels);
                            audioDeviceDroplist.selectedIndex = 0;

                            // Enable start button
                            startAudioButton.disabled = false;

                            // Update status message
                            audioClassificationResult.label = "Select a device and press Start";
                            confidenceScore.label = "Confidence: N/A";
                        }
                    } else {
                        // No devices returned
                        audioDeviceDroplist.labels = "No devices found";
                        console.log("No devices found.");
                        audioDeviceDroplist.selectedIndex = -1;

                        // Disable start button
                        startAudioButton.disabled = true;

                        // Update status message
                        audioClassificationResult.label = "No audio devices available";
                        confidenceScore.label = "Confidence: N/A";
                    }
                }).fail(function(jqXHR, textStatus, errorThrown) {
                    console.error("Error fetching audio devices: ", textStatus, errorThrown);
                    audioDeviceDroplist.labels = "Error loading devices";
                    audioDeviceDroplist.selectedIndex = -1;

                    // Disable start button
                    startAudioButton.disabled = true;

                    // Update status message
                    audioClassificationResult.label = "Failed to load audio devices";
                    confidenceScore.label = "Confidence: N/A";
                });
            };

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
                switch(state) {
                    case 'starting':
                        isClassifying = true;
                        startAudioButton.disabled = true;
                        stopAudioButton.disabled = false;
                        audioDeviceDroplist.disabled = true;
                        audioClassificationResult.label = "Starting classification...";
                        confidenceScore.label = "Confidence: ";
                        loadingSpinner.style.display = 'block';
                        break;

                    case 'started':
                        isClassifying = true;
                        startAudioButton.disabled = true;
                        stopAudioButton.disabled = false;
                        audioDeviceDroplist.disabled = true;
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
                        startAudioButton.disabled = false;
                        stopAudioButton.disabled = true;
                        audioDeviceDroplist.disabled = false;
                        loadingSpinner.style.display = 'none';
                        if (audioClassificationResult.label === "Starting classification..." ||
                            audioClassificationResult.label === "Stopping classification...") {
                            audioClassificationResult.label = "Classification stopped.";
                        }
                        break;

                    case 'error':
                        isClassifying = false;
                        startAudioButton.disabled = false;
                        stopAudioButton.disabled = true;
                        audioDeviceDroplist.disabled = false;
                        loadingSpinner.style.display = 'none';
                        break;
                }
            }

            // Start audio classification
            startAudioButton.addEventListener('click', function() {
                const selectedDevice = audioDeviceDroplist.selectedText;
                if (selectedDevice && selectedDevice !== "No devices found" && selectedDevice !== "Error loading devices") {
                    // Update UI to starting state
                    updateUIState('starting');

                    // Set up WebSocket first to ensure we catch early messages
                    setupAudioWebSocket();

                    // Send request to start classification
                    $.ajax({
                        url: "/start-audio-classification?device=" + encodeURIComponent(selectedDevice),
                        method: "GET",
                        dataType: "json",
                        timeout: 15000, // 15 seconds timeout
                        success: function(data) {
                            console.log("Start classification response:", data);
                            if (data.status === 'started') {
                                updateUIState('started');
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
                            console.error("Error starting audio classification:", jqXHR.responseText);
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
                } else {
                    audioClassificationResult.label = "Please select a valid audio device.";
                }
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