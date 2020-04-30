/*
 * gc global variable provides access to GUI Composer infrastructure components and project information.
 * For more information, please see the Working with Javascript guide in the online help.
 */
var gc = gc || {};
gc.services = gc.services || {};
var saveAsFilePath; // undefined initially
var regFileFormat = "CSV";

/*
 *  Boilerplate code for creating computed data bindings
 */
document.addEventListener('gc-databind-ready', function() {
    /*
     *   Add custom computed value databindings here, using the following method:
     *
     *   function gc.databind.registry.bind(targetBinding, modelBinding, [getter], [setter]);
     *
     *   param targetBinding - single binding string or expression, or array of binding strings for multi-way binding.
     *   param modelBinding - single binding string or expression, or array of binding strings for multi-way binding.
     *   param getter - (optional) - custom getter function for computing the targetBinding value(s) based on modelBinding value(s).
     *   param setter - (optional) - custom setter function for computing the modelBinding value(s) based on targetBinding value(s).
     */

    // For example, a simple computed values based on simple expression
    // gc.databind.registry.bind('widget.id.propertyName', "targetVariable == 1 ? 'binding is one' : 'binding is not one'");

    // Or a custom two-way binding with custome getter and setter functions.  (setter is optional)  (getter only indicates one-way binding)
    // gc.databind.registry.bind('widget.id.propertyName', "targetVariable", function(value) { return value*5/9 + 32; }, function(value) { (return value-32)*9/5; });

    // Event 1 to n bindings
    /*
    gc.databind.registry.bind('widget.date.value',
        // dependant bindings needed in order to compute the date, in name/value pairs.
        {
            weekday: 'widget.dayOfWeek.selectedText',
            day: 'widget.dayOfMonth.value',
            month: 'widget.month.selectedText',
            year: 'widget.year.value'
        },
        // getter for date computation
        function(values)
        {
            // compute and return the string value to bind to the widget with id 'date'
            return values.weekday + ', ' + values.month + ' ' + values.day + ', ' + values.year;
        }
    );
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
     *   param id - uniquely identifies the action, and should correspond to the action property of the menuaction widget.
     *   param runable - function that performs the custom action.
     *   param isAvailable - (optional) - function called when the menu action is about to appear.  Return false to disable the action, or true to enable it.
     *   param isVisible - (optional) - function called when the menu action is about to appear.  Return false to hide the action, or true to make it visible.
     */

    // For example,
    // gc.nav.registerAction('myCustomCloseAction', function() { window.close(); }, function() { return true; }, function() { return true; });

    // Alternatively, to programmatically disable a menu action at any time use:
    // gc.nav.disableAction('myCustomCloseAction);    then enable it again using:  gc.nav.enableAction('myCustomCloseAction');

    gc.nav.registerAction('show_introduction', function() {
        if ((templateObj) && (templateObj.$)) {
            templateObj.$.ti_widget_vtabcontainer.selectedIndex = 0;
            templateObj.$.ti_widget_menuitem.hideFullMenu();
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });
    gc.nav.registerAction('show_registers', function() {
        if ((templateObj) && (templateObj.$)) {
            templateObj.$.ti_widget_vtabcontainer.selectedIndex = 1;
            templateObj.$.ti_widget_menuitem.hideFullMenu();
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });
    gc.nav.registerAction('show_motor_control', function() {
        if ((templateObj) && (templateObj.$)) {
            templateObj.$.ti_widget_vtabcontainer.selectedIndex = 2;
            templateObj.$.ti_widget_menuitem.hideFullMenu();
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });

    gc.nav.registerAction('set_csv_format', function() {
        regFileFormat = "CSV";
        gc.nav.setActionChecked('set_csv_format', true);
        gc.nav.setActionChecked('set_json_format', false);
    }, function() {
        return  true; // isAvailable
    }, function() {
        return true; // isVisible
    });
    gc.nav.setActionChecked('set_csv_format', true);

    gc.nav.registerAction('set_json_format', function() {
        regFileFormat = "JSON";
        gc.nav.setActionChecked('set_csv_format', false);
        gc.nav.setActionChecked('set_json_format', true);
    }, function() {
        return true; // isAvailable
    }, function() {
        return true;  // isVisible
    });

    var saveText = function(fname, text, mime) {
        var blob = new Blob([text], {
            type: mime
        });
        saveAs(blob, fname);
        return false;
    };

    var getCsvForRegs = function(){
        var result = "register,address,value\n";
        var regPage = document.querySelector('#ti_widget_register_page');
        var regBlocks = regPage && regPage.regBlocks;
        if (regBlocks) {
            for (var b=0; b < regBlocks.length; b++) {
                var regBlock = regBlocks[b];
                for (var r=0; r < regBlock.registers.length; r++) {
                    var reg = regBlock.registers[r];
                    result += reg.name+","+reg.addr+","+reg.value+"\n";
                }
            }
        }
        return result;
    };

    /*
     * saveRegs saves the register values from the register page into either a CSV file 
     * format or a stringified JSON object and saves them to a file.  If isSaveAs
     * parameter is true (or the saveAsFilePath global has not been set), 
     * it shows the 'browseAndSave' dialog to allow the user to 
     * specify which folder to save the file into, otherwise it saves the contents to 
     * the file path specified by saveAsFilePath
     */
    var saveRegs = function(calledBy, isSaveAs) {
        var fileFilter;
        var regData;
        switch (regFileFormat) {
            case "CSV":
                regData = getCsvForRegs();
                fileFilter = ".csv";
                // make sure the file extension matches the regFileFormat
                if (saveAsFilePath) {
                    saveAsFilePath = saveAsFilePath.replace(".json", ".csv");
                }
                break;
            case "JSON":
            default:
                fileFilter = ".json";
                regData = JSON.stringify(templateObj.getRegisterData());
                // make sure the file extension matches the regFileFormat
                if (saveAsFilePath) {
                    saveAsFilePath = saveAsFilePath.replace(".csv", ".json");
                }
        }
        var fileName = saveAsFilePath;
        if (!fileName) {
            fileName = "regs" + fileFilter;
        }
        if (gc.desktop.isDesktop()) {
            if (isSaveAs || !saveAsFilePath ) {
                gc.File.browseAndSave(regData, fileName, fileFilter, function (result, errorInfo) {
                    if (errorInfo) {
                        templateObj.logMsgAndNotify("error", calledBy + " " + errorInfo, false, true);
                    } else if ((result) && (result.localPath)) {
                        saveAsFilePath = result.localPath;
                        gc.nav.enableAction('save_registers');
                        templateObj.$.ti_widget_eventlog_view.log("info", "Registers saved to " + result.localPath);
                    }
                });
            } else {
                gc.File.save(regData, {"localPath": saveAsFilePath}, null, function(errorInfo) {
                    if (errorInfo) {
                        templateObj.logMsgAndNotify("error", "Save Registers As: " + errorInfo, false, true);
                    } else {
                        templateObj.$.ti_widget_eventlog_view.log("info", "Registers saved to " + saveAsFilePath);
                    }
                });
            }
        } else {
            saveText(fileName, regData, 'text/plain;charset=utf-8');
            templateObj.$.ti_widget_eventlog_view.log("info", calledBy+": "+fileName+" file downloaded");
        }
    };

    /*
     * getRegsFromCsv decodes the string passed in as a parameter as the contents of a CSV file
     * and updates the registers in the regBlocks object with the values from the CSV file.
     * NOTE that the target is not updated with these values by this function.  To update the
     * target registers, please call setRegisterData.
     */
    getRegsFromCsv = function(csvContents) {
        var lines = csvContents.split('\n');
        lines.shift(); // discard the title row
        // create a map of the csv values
        var csvMap = new Map();
        for (var i=0; i < lines.length; i++){
            var regInfo = lines[i].split(",");
            var name = regInfo[0].trim().toLowerCase();
            var addr = +regInfo[1];
            var value = +regInfo[2];
            var key = name+"@"+addr;
            csvMap.set(key,value);
        }
        var regPage = document.querySelector('#ti_widget_register_page');
        var regBlocks = regPage && regPage.regBlocks;
        if (regBlocks) {
            for (var b = 0; b < regBlocks.length; b++) {
                var regBlock = regBlocks[b];
                for (var r = 0; r < regBlock.registers.length; r++) {
                    var reg = regBlock.registers[r];
                    var regAdrs = +reg.addr;
                    var regKey = reg.name.trim().toLowerCase() + "@"+regAdrs;
                    if (csvMap.has(regKey)){
                        reg.value = csvMap.get(regKey);
                    }
                }
            }
        }
        return regBlocks;
    };

    gc.nav.registerAction('load_registers', function() {
        if ((templateObj) && (templateObj.$)) {
            gc.File.browseAndLoad(null, {
                bin: false
            }, function(contents, properties) {
                var regData = null;
                try {
                    var fileType =  regFileFormat;
                    var dotIndex = (properties && properties.name) ? properties.name.lastIndexOf(".") : -1;
                    if (dotIndex >= 0 && dotIndex < properties.name.length) {
                        fileType = properties.name.substring(dotIndex + 1).toUpperCase();
                        if (fileType !== "JSON" && fileType !== "CSV") {
                            fileType = regFileFormat;
                        }
                    }
                    switch (fileType) {
                        case "CSV":
                            regData = getRegsFromCsv(contents);
                            break;
                        default:
                        case "JSON":
                            regData = JSON.parse(contents);
                            if (typeof regData === "string") {
                                regData = JSON.parse(regData);
                            }
                            if (regData && regData.data) {
                                regData = regData.data;
                            }
                            break;
                    }
                    if (regData) {
                        if (templateObj.setRegisterData(regData)) {
                            templateObj.$.ti_widget_eventlog_view.log("info", "Load Registers: loaded from :" + properties.localPath);
                        }
                    }
                } catch (ex) {
                    templateObj.logMsgAndNotify("error", "Load Registers: Could not parse " + regFileFormat + " file:" + ex, false, true);
                }

            });
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });

    gc.nav.registerAction('save_registers', function() {
        if ((templateObj) && (templateObj.$)) {
            saveRegs("Save Registers", false);
        }
    }, function() {
        return saveAsFilePath;  // isAvailable
    }, function() {
        return saveAsFilePath;  // isVisible
    });
    gc.nav.disableAction('save_registers');
    gc.nav.registerAction('save_registers_as', function() {
        if ((templateObj) && (templateObj.$)) {
            saveRegs("Save Registers As", true);
        }
    }, function() {
        return true;
    }, function() {
        return true;
    });

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
var registerModel;


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
            gc.nav.registerChecklistActions('fileFormat.', fileFormat,
                {
                    onSelectionHandler: function(formatName, detail) {
                        regFileFormat = formatName;
                        gc.localStorage.setItem('fileFormat', formatName);
                    },

                    onIsAvailableHandler: function() {
                        return false;
                    }
                }
                , gc.localStorage.getItem('fileFormat') || regFileFormat, false);
            registerModel = Polymer.dom(document).querySelector('ti-model-register');
            if (registerModel) {
                registerModel.addEventListener("usbHidModelReceivingData", function (event) {
                    templateObj.logMsgAndNotify("debug", "Receiving data from target.", false, false);
                });
            } else {
                console.log("no ti-model-register component found in this application!")
            }

            // Now that the template has been stamped, you can use 'automatic node finding' $ syntax to access widgets.
            // e.g. to access a widget with an id of 'widget_id' you can use templateObj.$.widgetId

            // open the eventlog viewer when the user clicks on the book icon in the status bar:
            templateObj.$.ti_widget_eventlog_view.closeView(); // ensure that view states are in sync with application
            templateObj.$.ti_widget_statusbar.addEventListener("status-icon-clicked", function(event) {
                // TODO: add whatever actions you wish to perform in response to the appStatusIcon in the statusbar
                // being clicked.  See event.details for info on the icon and app status
            });

            // Expand vtabcontainer nav bar when user clicks on menu icon or 'Menu' label
            templateObj.toggleMenu = function(event){
                templateObj.$.ti_widget_vtabcontainer.isExpanded = !templateObj.$.ti_widget_vtabcontainer.isExpanded;
            };
            templateObj.$.ti_widget_icon_button_menu.addEventListener('click',templateObj.toggleMenu);
            templateObj.$.ti_widget_label_menu.addEventListener('click',templateObj.toggleMenu);
            var regPage = document.querySelector('#ti_widget_register_page');
            if (regPage) {
                // get the active device name from the register page
                templateObj.$.ti_widget_menubar.productName = regPage.deviceName;

                // called when the user clicks the Write Register button:
                regPage.addEventListener("write_reg", function (event) {
                    var detailsObj = event.detail;
                    templateObj.logMsgAndNotify(detailsObj.statusType, detailsObj.statusMsg, false, true);
                }.bind(this));

                // called when the user clicks the Write All Registers button:
                regPage.addEventListener("write_regs", function (event) {
                    var detailsObj = event.detail;
                    templateObj.logMsgAndNotify(detailsObj.statusType, detailsObj.statusMsg, false, true);
                }.bind(this));

                // called when the user clicks the Read Register button:
                regPage.addEventListener("read_reg", function (event) {
                    var detailsObj = event.detail;
                    templateObj.logMsgAndNotify(detailsObj.statusType, detailsObj.statusMsg, false, true);
                }.bind(this));

                // called when the user clicks the Read All Registers button:
                regPage.addEventListener("read_regs", function (event) {
                    var detailsObj = event.detail;
                    templateObj.logMsgAndNotify(detailsObj.statusType, detailsObj.statusMsg, false, true);
                }.bind(this));

                // called when the user changes the immediate / deferred mode
                regPage.addEventListener("update_mode_changed", function (event) {
                    var detailsObj = event.detail;
                    templateObj.$.ti_widget_eventlog_view.log("info", "request to change update mode to " + detailsObj.mode.label);
                }.bind(this));

                // called when the user changes the auto read refresh rate
                regPage.addEventListener("auto_read_config_changed", function (event) {
                    var detailsObj = event.detail;
                    templateObj.$.ti_widget_eventlog_view.log("info", "request to change automatic read interval to " + detailsObj.autoRead.label);
                }.bind(this));
            }

            var usbHidService = gc.services['ti-service-usbhid'];
            if (usbHidService) {
                usbHidService.addEventListener('usbHidError', function(event) {
                    var detailsObj = event.detail;
                    templateObj.logMsgAndNotify("error", detailsObj.info + ": " + detailsObj.error, false, true);
                })
            }

            templateObj.getRegisterData = function() {
                var regData = {
                    signature: "register-data",
                    data: []
                };
                var regPage = document.querySelector('#ti_widget_register_page');
                var regBlocks = regPage && regPage.regBlocks;
                if (regBlocks) {
                    for (var i = 0; i < regBlocks.length; i++) {
                        var registersData = [];
                        for (var r = 0; r < regBlocks[i].registers.length; r++) {
                            var reg = regBlocks[i].registers[r];
                            var regName = reg.name.toLowerCase();
                            regName = regName.replace(/&/g, "_");
                            var item = {
                                idx: r,
                                id: regName,
                                value: reg.value
                            };
                            registersData.push(item);
                        }
                        regData.data.push(registersData);
                    }
                }
                return regData;
            };

            templateObj.setRegisterData = function(jsonObj) {
                var ok = true;
                var regData = jsonObj;
                var regPage = document.querySelector('#ti_widget_register_page');
                var regBlocks = regPage && regPage._registerView.regBlocks;
                if (regBlocks) {
                    for (var b = 0; b < Math.min(regData.length, regBlocks.length); b++) {
                        var idx = 0;
                        var dRegName = "";
                        var dBlock = regData[b];
                        var dNumRegs = dBlock.length;
                        if (dBlock.registers) {
                            dNumRegs = dBlock.registers.length;
                        }
                        for (var r = 0; r < regBlocks[b].registers.length; r++) {
                            var reg = regBlocks[b].registers[r];
                            var regName = reg.name.toLowerCase();
                            var dReg = null;
                            if (dBlock.registers && dBlock.registers.length > idx) {
                                // from .csv file
                                dReg = dBlock.registers[idx];
                                dRegName = dReg.name;
                            } else if (dBlock.length && dBlock.length > idx) {
                                // from .json file
                                dReg = dBlock[idx];
                                dRegName = dReg.id;
                            }
                            if (dRegName && dRegName.toLowerCase() === regName) {
                                idx++;
                                reg.value = dReg.value;
                                if (!regPage.isDeferredWrite) {
                                    reg.valueLastWritten = reg.value;
                                    var intValue = gc.utils.getValueFromHexString("" + reg.value);
                                    regPage.writeValueToTargetRegister(intValue, null, b, r);
                                } else {
                                    regPage._registerView.refreshRegValue(b,r);
                                }
                            }
                        }
                        if (idx < dNumRegs) {
                            var msg = "load registers failed: register name mismatch (" + dRegName + " not found.)";
                            templateObj.logMsgAndNotify("warning", msg, false, true);
                            ok = false;
                            break;
                        }
                    }
                    if (regPage.isDeferredWrite){
                        regPage._registerView._deferredAccessCtr++; // update the page
                    }
                    regPage.refreshGrid();
                }
                return ok;
            };

            templateObj.showToastMsg = function(msgType, msg, durationInSeconds) {
                templateObj.$.ti_widget_toast.hideToast();
                templateObj.$.ti_widget_toast.message = msg;
                switch (msgType.toLowerCase()) {
                    case "error":
                        templateObj.$.ti_widget_toast.iconName = "error";
                        templateObj.$.ti_widget_toast.backgroundColor = "red";
                        break;
                    case "warning":
                        templateObj.$.ti_widget_toast.iconName = "warning";
                        templateObj.$.ti_widget_toast.backgroundColor = "orange";
                        break;
                    default:
                        templateObj.$.ti_widget_toast.iconName = "check";
                        templateObj.$.ti_widget_toast.backgroundColor = "teal";
                        break;
                }
                templateObj.$.ti_widget_toast.duration = +durationInSeconds;
                templateObj.$.ti_widget_toast.showToast();
                console.log("toast msg [" + msgType + "]: " + msg);
            };

            templateObj.logMsgAndNotify = function(msgType, msg, showInStatusBar, showInToast) {
                var iconName = templateObj.$.ti_widget_eventlog_view.iconForItem(msgType);
                if (showInStatusBar) {
                    var statusBar = document.querySelector('ti-widget-statusbar');
                    if (statusBar) {
                        statusBar.appStatusText = msg;
                        statusBar.appStatusTooltip = msg;
                        statusBar.setIcon(iconName);
                    }
                }
                if (showInToast) {
                    templateObj.showToastMsg(msgType, msg, 4);
                }
                templateObj.$.ti_widget_eventlog_view.log(msgType, msg);
            };
            if (templateObj.$.ti_widget_label_learnmore) {
                templateObj.$.ti_widget_label_learnmore.addEventListener("click", function (event) {
                    templateObj.$.ti_widget_label_productname.fontColor = "#444";
                    templateObj.$.ti_widget_label_productdesc.fontColor = "#444";
                    templateObj.$.ti_widget_label_learnmore.setAttribute('hidden', true);
                    templateObj.$.ti_widget_label_showless.removeAttribute('hidden');
                    templateObj.$.ti_widget_vtab_home.style.backgroundColor = "white";
                    templateObj.$.ti_widget_tabcontainer.selectedIndex = 1;
                });
            }
            if (templateObj.$.ti_widget_label_showless) {
                templateObj.$.ti_widget_label_showless.addEventListener("click", function (event) {
                    templateObj.$.ti_widget_label_productname.fontColor = "white";
                    templateObj.$.ti_widget_label_productdesc.fontColor = "white";
                    templateObj.$.ti_widget_label_learnmore.removeAttribute('hidden');
                    templateObj.$.ti_widget_label_showless.setAttribute('hidden', true);
                    templateObj.$.ti_widget_vtab_home.style.backgroundColor = "rgb(20, 140, 156)";
                    templateObj.$.ti_widget_tabcontainer.selectedIndex = 0;
                });
            }
            if (templateObj.$.ti_widget_button_connect) {
                templateObj.$.ti_widget_button_connect.addEventListener("click",function(event){
                    if (gc.connectionManager){
                        gc.connectionManager.connect();
                    }
                });
            }
            
            templateObj.$.load_0.addEventListener('click', () => {
                const app_body = '{"core0":{"input":{"application":' + app_list_0.getSelectedIndex() + '}}}';
                const freq_body = '{"core0":{"input":{"frequency":' + freq_list_0.getSelectedIndex() + '}}}';
                fetch('http://1.1.1.1:8081/oob_data.json', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    referrerPolicy: 'no-referrer-when-downgrade',
                    body: app_body + freq_body
                })
                .then((response) => response.json())
                .catch((error) => console.error(error));
            });

            templateObj.$.load_1.addEventListener('click', () => {
                const app_body = '{"core1":{"input":{"application":' + app_list_1.getSelectedIndex() + '}}}';
                const freq_body = '{"core1":{"input":{"frequency":' + freq_list_1.getSelectedIndex() + '}}}';
                fetch('http://1.1.1.1:8081/oob_data.json', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    referrerPolicy: 'no-referrer-when-downgrade',
                    body: app_body + freq_body
                })
                .then((response) => response.json())
                .catch((error) => console.error(error));
            });

            templateObj.$.load_2.addEventListener('click', () => {
                const app_body = '{"core2":{"input":{"application":' + app_list_2.getSelectedIndex() + '}}}';
                const freq_body = '{"core2":{"input":{"frequency":' + freq_list_2.getSelectedIndex() + '}}}';
                fetch('http://1.1.1.1:8081/oob_data.json', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    referrerPolicy: 'no-referrer-when-downgrade',
                    body: app_body + freq_body
                })
                .then((response) => response.json())
                .catch((error) => console.error(error));
            });

            templateObj.$.load_3.addEventListener('click', () => {
                const app_body = '{"core3":{"input":{"application":' + app_list_3.getSelectedIndex() + '}}}';
                const freq_body = '{"core3":{"input":{"frequency":' + freq_list_3.getSelectedIndex() + '}}}';
                fetch('http://1.1.1.1:8081/oob_data.json', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    referrerPolicy: 'no-referrer-when-downgrade',
                    body: app_body + freq_body
                })
                .then((response) => response.json())
                .catch((error) => console.error(error));
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