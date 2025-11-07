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

            // ===== AUDIO CLASSIFICATION - SIMPLIFIED VERSION =====
            console.log("=== Audio Classification Init ===");

            const fetchDevicesButton = templateObj.$.fetch_devices_button;
            const startAudioButton = templateObj.$.start_audio_button;
            const stopAudioButton = templateObj.$.stop_audio_button;
            const audioClassificationResult = templateObj.$.audio_classification_result;
            const confidenceScore = templateObj.$.confidence_score;

            let selectedDevice = null;
            let audioDevices = [];

            console.log("Fetch button:", fetchDevicesButton ? "OK" : "MISSING");
            console.log("Start button:", startAudioButton ? "OK" : "MISSING");

            // Simple button click to fetch devices
            if (fetchDevicesButton) {
                fetchDevicesButton.addEventListener('click', function() {
                    console.log(">>> FETCH DEVICES BUTTON CLICKED <<<");
                    fetchAudioDevices();
                });
            }

            function fetchAudioDevices() {
                console.log("fetchAudioDevices() called");

                var container = document.getElementById('device_list_container');
                if (!container) {
                    console.error("ERROR: device_list_container not found!");
                    return;
                }

                container.innerHTML = '<p style="color: blue;">Loading devices...</p>';

                console.log("Making AJAX call to /audio-devices");

                $.ajax({
                    url: '/audio-devices',
                    type: 'GET',
                    dataType: 'text',
                    success: function(response) {
                        console.log("SUCCESS! Response:", response);
                        displayDevices(response);
                    },
                    error: function(xhr, status, error) {
                        console.error("ERROR!", status, error);
                        console.error("Response:", xhr.responseText);
                        container.innerHTML = '<p style="color: red;">Error: ' + error + '</p>';
                    }
                });
            }

            function displayDevices(responseText) {
                console.log("displayDevices() called with:", responseText);

                var container = document.getElementById('device_list_container');
                var lines = responseText.trim().split('\n');

                console.log("Parsed lines:", lines);

                if (lines.length === 0 || lines[0].toLowerCase().includes('error') ||
                    lines[0].toLowerCase().includes('no audio')) {
                    container.innerHTML = '<p>No audio devices found</p>';
                    return;
                }

                audioDevices = lines;
                var html = '<div style="border: 1px solid #ccc; padding: 10px;">';

                for (var i = 0; i < lines.length; i++) {
                    var deviceName = lines[i].trim();
                    html += '<div style="margin: 10px 0; padding: 10px; background: #f0f0f0; border-radius: 5px;">';
                    html += '<span style="font-weight: bold;">' + deviceName + '</span>';
                    html += '<button onclick="window.selectDevice(\'' + deviceName + '\')" style="margin-left: 10px; padding: 5px 15px; background: #148C9C; color: white; border: none; border-radius: 3px; cursor: pointer;">Select</button>';
                    html += '</div>';
                }

                html += '</div>';
                container.innerHTML = html;
            }

            // Global function for button onclick
            window.selectDevice = function(deviceName) {
                console.log("Device selected:", deviceName);
                selectedDevice = deviceName;
                startAudioButton.disabled = false;
                audioClassificationResult.label = "Ready: " + deviceName;
            };

            // WebSocket for audio classification results
            let wsAudio = null;
            let isClassifying = false;
            let reconnectTimeout = null;
            let reconnectAttempts = 0;
            const MAX_RECONNECT_ATTEMPTS = 5;
            const RECONNECT_INTERVAL = 1000; // 1 second between attempts

            // Set up WebSocket immediately (persistent connection)
            setupAudioWebSocket();

            // Function to set up WebSocket for audio classification results
            function setupAudioWebSocket() {
                console.log("[Audio WebSocket] Setting up connection");
                clearTimeout(reconnectTimeout); // Clear any pending reconnects

                // Close any existing WebSocket connection
                if (wsAudio) {
                    try {
                        console.log("[Audio WebSocket] Closing existing connection");
                        wsAudio.onclose = null; // Prevent onclose handler during intentional close
                        wsAudio.close();
                    } catch (e) {
                        console.error("[Audio WebSocket] Error closing socket:", e);
                    }
                    wsAudio = null;
                }

                try {
                    // Create new WebSocket connection
                    wsAudio = new WebSocket("ws://" + window.location.hostname + ":" + window.location.port + "/audio");

                    wsAudio.onopen = function() {
                        console.log("[Audio WebSocket] Connected successfully");
                        reconnectAttempts = 0; // Reset reconnect counter on successful connection
                    };

                    wsAudio.onmessage = function(event) {
                        try {
                            const result = JSON.parse(event.data);
                            console.log("[Audio WebSocket] Received:", result);

                            // Handle different message types
                            if (result.status === 'connected') {
                                console.log("[Audio WebSocket] Initial connection message received");
                            } else if (result.status === 'stopped') {
                                console.log("[Audio WebSocket] Classification stopped");
                                audioClassificationResult.label = "Classification stopped";
                                startAudioButton.disabled = false;
                                stopAudioButton.disabled = true;
                                isClassifying = false;
                            } else if (result.error) {
                                console.error("[Audio WebSocket] Error:", result.error);
                                audioClassificationResult.label = "Error: " + result.error;
                                if (confidenceScore) confidenceScore.label = "Confidence: N/A";
                                startAudioButton.disabled = false;
                                stopAudioButton.disabled = true;
                                isClassifying = false;
                            } else if (result.class) {
                                // Classification result - LIVE UPDATES!
                                console.log("[Audio WebSocket] Classification result:", result.class);
                                audioClassificationResult.label = result.class;

                                if (confidenceScore && result.confidence !== undefined) {
                                    const confidence = parseFloat(result.confidence);
                                    const confidencePercent = isNaN(confidence) ? 0 : confidence * 100;
                                    confidenceScore.label = "Confidence: " + confidencePercent.toFixed(1) + "%";
                                }
                            }
                        } catch (e) {
                            console.error("[Audio WebSocket] Error parsing message:", e, "Data:", event.data);
                        }
                    };

                    wsAudio.onclose = function(event) {
                        console.log("[Audio WebSocket] Connection closed", event ? `code: ${event.code}` : '');

                        // If we're classifying, show connection lost
                        if (isClassifying) {
                            audioClassificationResult.label = "Connection lost - reconnecting...";
                        }

                        wsAudio = null;

                        // Auto reconnect unless max attempts reached
                        if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                            reconnectAttempts++;
                            const delay = RECONNECT_INTERVAL * reconnectAttempts;
                            console.log(`[Audio WebSocket] Reconnecting in ${delay}ms (attempt ${reconnectAttempts})`);

                            reconnectTimeout = setTimeout(setupAudioWebSocket, delay);
                        } else if (isClassifying) {
                            console.error("[Audio WebSocket] Max reconnection attempts reached");
                            audioClassificationResult.label = "Connection lost";
                            startAudioButton.disabled = false;
                            stopAudioButton.disabled = true;
                            isClassifying = false;
                        }
                    };

                    wsAudio.onerror = function(error) {
                        console.error("[Audio WebSocket] Connection error:", error);
                        // Don't reset UI here - onclose will be called after error and handle it
                    };
                } catch (e) {
                    console.error("[Audio WebSocket] Error creating WebSocket:", e);
                    wsAudio = null;
                }
            }

            // Start button handler
            if (startAudioButton) {
                startAudioButton.addEventListener('click', function() {
                    console.log(">>> START BUTTON CLICKED <<<");
                    if (!selectedDevice) {
                        alert("Please select a device first!");
                        return;
                    }

                    console.log("[Audio] Starting classification with device:", selectedDevice);

                    // WebSocket is already set up (persistent connection)
                    // Just ensure it's connected or reconnect if needed
                    if (!wsAudio) {
                        console.log("[Audio] WebSocket not connected, reconnecting");
                        setupAudioWebSocket();
                    }

                    // Update UI
                    audioClassificationResult.label = "Starting...";
                    startAudioButton.disabled = true;
                    stopAudioButton.disabled = true;

                    // First try stopping any existing classification
                    $.ajax({
                        url: '/stop-audio-classification',
                        type: 'GET',
                        complete: function() {
                            // Start classification after cleanup - no matter what happened with stop
                            $.ajax({
                                url: '/start-audio-classification?device=' + encodeURIComponent(selectedDevice),
                                type: 'GET',
                                success: function(response) {
                                    console.log("[Audio] Start SUCCESS:", response);
                                    audioClassificationResult.label = "Listening...";
                                    stopAudioButton.disabled = false;
                                    isClassifying = true;
                                },
                                error: function(xhr, status, error) {
                                    console.error("[Audio] Start ERROR:", error);
                                    let errorMessage = error;

                                    // Handle "already running" error specifically
                                    if (xhr.responseText && xhr.responseText.indexOf('already running') !== -1) {
                                        console.log("[Audio] Classification already running - treating as success");
                                        audioClassificationResult.label = "Listening... (reconnected)";
                                        stopAudioButton.disabled = false;
                                        isClassifying = true;
                                        return; // Exit early - we're treating this as success
                                    }

                                    audioClassificationResult.label = "Error: " + errorMessage;
                                    startAudioButton.disabled = false;
                                    stopAudioButton.disabled = true;
                                    isClassifying = false;
                                    if (wsAudio) {
                                        wsAudio.close();
                                        wsAudio = null;
                                    }
                                }
                            });
                        }
                    });
                });
            }

            // Stop button handler
            if (stopAudioButton) {
                stopAudioButton.addEventListener('click', function() {
                    console.log(">>> STOP BUTTON CLICKED <<<");

                    audioClassificationResult.label = "Stopping...";
                    stopAudioButton.disabled = true;

                    $.ajax({
                        url: '/stop-audio-classification',
                        type: 'GET',
                        success: function(response) {
                            console.log("[Audio] Stop SUCCESS:", response);
                            audioClassificationResult.label = "Classification stopped";
                            startAudioButton.disabled = false;
                            stopAudioButton.disabled = true;
                            isClassifying = false;
                            // DON'T close WebSocket - keep it open for next classification
                            // if (wsAudio) {
                            //     wsAudio.close();
                            //     wsAudio = null;
                            // }
                        },
                        error: function(xhr, status, error) {
                            console.error("[Audio] Stop ERROR:", error);
                            audioClassificationResult.label = "Error stopping";
                            startAudioButton.disabled = false;
                            stopAudioButton.disabled = true;
                            isClassifying = false;
                        }
                    });
                });
            }

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