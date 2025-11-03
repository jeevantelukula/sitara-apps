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
    templateObj = document.querySelector('#template_obj');

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
                templateObj.$.ti_widget_vtabcontainer.isExpanded = !templateObj.$.ti_widget_vtabcontainer.isExpanded;
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

            // Audio Classification specific logic
            const audioClassificationVtab = templateObj.$.ti_widget_vtab_audio_classification;
            const audioDeviceDroplist = templateObj.$.audio_device_droplist;
            const startAudioButton = templateObj.$.start_audio_button;
            const stopAudioButton = templateObj.$.stop_audio_button;
            const audioClassificationResult = templateObj.$.audio_classification_result;

            let wsAudio;

            // Fetch devices when the audio classification tab is selected
            templateObj.$.ti_widget_vtabcontainer.addEventListener('selected-item-changed', function(event) {
                console.log("selected-item-changed event fired.");
                console.log("event.detail.value:", event.detail.value);
                console.log("audioClassificationVtab:", audioClassificationVtab);
                if (event.detail.value === audioClassificationVtab) {
                    console.log("Audio Classification tab selected. Calling fetchAudioDevices().");
                    fetchAudioDevices();
                }
            });

            // Function to fetch audio devices
            const fetchAudioDevices = function() {
                console.log("fetchAudioDevices() called.");

            // Start audio classification
            startAudioButton.addEventListener('click', function() {
                const selectedDevice = audioDeviceDroplist.selectedText;
                if (selectedDevice && selectedDevice !== "No devices found" && selectedDevice !== "Error loading devices") {
                    startAudioButton.disabled = true;
                    stopAudioButton.disabled = false;
                    audioClassificationResult.label = "Starting classification...";

                    $.get("/start-audio-classification?device=" + encodeURIComponent(selectedDevice), function(data) {
                        console.log(data);
                        audioClassificationResult.label = "Classification started. Waiting for results...";

                        // Establish WebSocket connection for live results
                        wsAudio = new WebSocket("ws://" + window.location.hostname + ":8083");

                        wsAudio.onopen = function() {
                            console.log("Audio Classification WebSocket connected.");
                        };

                        wsAudio.onmessage = function(event) {
                            audioClassificationResult.label = "Class: " + event.data;
                        };

                        wsAudio.onclose = function() {
                            console.log("Audio Classification WebSocket disconnected.");
                            audioClassificationResult.label = "Classification stopped.";
                            startAudioButton.disabled = false;
                            stopAudioButton.disabled = true;
                        };

                        wsAudio.onerror = function(error) {
                            console.error("Audio Classification WebSocket error: ", error);
                            audioClassificationResult.label = "Error in classification stream.";
                            startAudioButton.disabled = false;
                            stopAudioButton.disabled = true;
                        };

                    }).fail(function(jqXHR, textStatus, errorThrown) {
                        console.error("Error starting audio classification: ", textStatus, errorThrown);
                        audioClassificationResult.label = "Failed to start classification.";
                        startAudioButton.disabled = false;
                        stopAudioButton.disabled = true;
                    });
                } else {
                    audioClassificationResult.label = "Please select a valid audio device.";
                }
            });

            // Stop audio classification
            stopAudioButton.addEventListener('click', function() {
                $.get("/stop-audio-classification", function(data) {
                    console.log(data);
                    if (wsAudio) {
                        wsAudio.close();
                    }
                    audioClassificationResult.label = "Stopping classification...";
                    startAudioButton.disabled = false;
                    stopAudioButton.disabled = true;
                }).fail(function(jqXHR, textStatus, errorThrown) {
                    console.error("Error stopping audio classification: ", textStatus, errorThrown);
                    audioClassificationResult.label = "Failed to stop classification.";
                    startAudioButton.disabled = false;
                    stopAudioButton.disabled = true;
                });
            });

        }, 1);

    });
};

templateObj = document.querySelector('#template_obj');
if (templateObj) {
    init();
} else {
    document.addEventListener('DOMContentLoaded', init);
}