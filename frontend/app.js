document.addEventListener('DOMContentLoaded', initializePage);

const API_BASE_URL = 'http://localhost:5000'; // Assuming backend runs on this port

// DOM Element References
let trainForm, modelTypeSelect, conversionTargetSelect, epochsInput, imgszInput, deviceSelect;
let startTrainButton, logOutput, jobStatusDisplay, bestModelPathDisplay, convertedModelPathDisplay;
let dataYamlPathInput, runNameInput;

let eventSource = null; // For Server-Sent Events (log streaming)
let pollingIntervalId = null; // To store ID from setInterval or similar for polling
let isPolling = false; // Flag to manage polling state

function initializePage() {
    // Get DOM elements
    trainForm = document.getElementById('trainForm');
    dataYamlPathInput = document.getElementById('data_yaml_path');
    modelTypeSelect = document.getElementById('model_type');
    epochsInput = document.getElementById('epochs');
    imgszInput = document.getElementById('imgsz');
    deviceSelect = document.getElementById('device');
    conversionTargetSelect = document.getElementById('conversion_target');
    runNameInput = document.getElementById('run_name');
    startTrainButton = document.getElementById('startTrainButton');
    logOutput = document.getElementById('logOutput');
    jobStatusDisplay = document.getElementById('jobStatus');
    bestModelPathDisplay = document.getElementById('bestModelPath');
    convertedModelPathDisplay = document.getElementById('convertedModelPath');

    // Fetch configuration
    fetch(`${API_BASE_URL}/config_options`)
        .then(response => response.json())
        .then(config => {
            populateSelect(modelTypeSelect, config.model_types);
            populateSelect(conversionTargetSelect, config.conversion_targets, true, "None (Train Only)"); // Add a "None" option
            
            epochsInput.value = config.default_training_parameters.epochs;
            imgszInput.value = config.default_training_parameters.imgsz;
            // Ensure device default is selected if it exists in the dropdown
            const deviceDefault = config.default_training_parameters.device;
            if (Array.from(deviceSelect.options).some(opt => opt.value === deviceDefault)) {
                deviceSelect.value = deviceDefault;
            }
        })
        .catch(error => {
            console.error('Error fetching config options:', error);
            appendToLog('Error: Could not load configuration from backend.');
        });

    // Fetch initial job status
    updateJobStatusFromServer(); // This will also handle UI updates based on status

    // Add event listeners
    trainForm.addEventListener('submit', handleTrainFormSubmit);
}

function populateSelect(selectElement, options, allowEmpty = false, emptyText = "Select an option") {
    selectElement.innerHTML = ''; // Clear existing options
    if (allowEmpty) {
        const emptyOption = document.createElement('option');
        emptyOption.value = "";
        emptyOption.textContent = emptyText;
        selectElement.appendChild(emptyOption);
    }
    options.forEach(optionValue => {
        const option = document.createElement('option');
        option.value = optionValue;
        option.textContent = optionValue;
        selectElement.appendChild(option);
    });
}

function handleTrainFormSubmit(event) {
    event.preventDefault();
    startTrainButton.disabled = true;
    appendToLog('Initiating training request...', true); // Clear previous logs
    updateStatusDisplay('submitting', '-', '-');

    const formData = {
        data_yaml_path: dataYamlPathInput.value,
        model_type: modelTypeSelect.value,
        epochs: parseInt(epochsInput.value, 10),
        imgsz: parseInt(imgszInput.value, 10),
        device: deviceSelect.value,
        conversion_target: conversionTargetSelect.value || null, // Send null if empty
        run_name: runNameInput.value || null
    };

    fetch(`${API_BASE_URL}/train_and_convert`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(formData)
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'started' || data.status === 'running') { // Backend might return 'running' if it was already so
            appendToLog(`Training job started successfully. PID: ${data.job_pid}. Log file: ${data.log_file}`);
            updateStatusDisplay(data.job_details.status, data.job_details.best_pt_path, data.job_details.converted_model_path);
            startLogStream();
            startPollingJobStatus(); // Start polling
        } else if (data.status === 'error' && data.message.includes("already in progress")) {
            appendToLog(`Error: ${data.message}`);
            appendToLog("Attempting to connect to existing job's log stream and status...");
            updateJobStatusFromServer(); // Fetch current status, which might trigger log stream and polling
        } 
        else {
            throw new Error(data.message || 'Failed to start training job.');
        }
    })
    .catch(error => {
        console.error('Error starting training:', error);
        appendToLog(`Error: ${error.message}`);
        updateStatusDisplay('error', '-', '-');
        startTrainButton.disabled = false;
    });
}

function startLogStream() {
    if (eventSource) {
        eventSource.close();
    }
    eventSource = new EventSource(`${API_BASE_URL}/training_log_stream`);
    appendToLog('Connecting to log stream...');

    eventSource.onmessage = (event) => {
        appendToLog(event.data);
    };

    eventSource.addEventListener('job_terminated', (event) => {
        appendToLog(`Job terminated event received. Final status: ${event.data}`);
        if (eventSource) {
            eventSource.close();
            eventSource = null;
        }
        stopPollingJobStatus(); // Stop polling as job is terminated
        updateJobStatusFromServer(); // Fetch final status details
    });

    eventSource.onerror = (error) => {
        console.error('EventSource failed:', error);
        appendToLog('Log stream connection error or closed.');
        // Don't close eventSource here if job might still be running.
        // Polling will eventually determine final state.
        // If job is no longer running, this error is expected after the 'job_terminated' event.
        if (currentJobStatus !== 'running') { // currentJobStatus needs to be a global or accessible variable
             if (eventSource) {
                eventSource.close();
                eventSource = null;
             }
        }
    };
}
let currentJobStatus = 'idle'; // Keep track of job status for SSE error handler

function appendToLog(message, clear = false) {
    if (clear) {
        logOutput.innerHTML = '';
    }
    const escapedMessage = message.replace(/</g, "&lt;").replace(/>/g, "&gt;");
    logOutput.innerHTML += escapedMessage + '\n';
    logOutput.scrollTop = logOutput.scrollHeight;
}

function updateStatusDisplay(status, bestPath, convertedPath) {
    currentJobStatus = status; // Update global status tracker
    jobStatusDisplay.textContent = status || 'N/A';
    bestModelPathDisplay.textContent = bestPath || '-';
    convertedModelPathDisplay.textContent = convertedPath || '-';

    if (status === 'running') {
        startTrainButton.disabled = true;
        jobStatusDisplay.className = ''; // Default style
    } else if (status === 'completed') {
        startTrainButton.disabled = false;
        jobStatusDisplay.className = 'success';
    } else if (status === 'failed' || status === 'error') {
        startTrainButton.disabled = false;
        jobStatusDisplay.className = 'error';
    } else {
        startTrainButton.disabled = false;
        jobStatusDisplay.className = '';
    }
}

function updateJobStatusFromServer() {
    fetch(`${API_BASE_URL}/training_status`)
        .then(response => response.json())
        .then(data => {
            updateStatusDisplay(data.status, data.best_pt_path, data.converted_model_path);
            
            if (data.status === 'running') {
                if (!eventSource || eventSource.readyState === EventSource.CLOSED) {
                    appendToLog("Job is running. Starting log stream...");
                    startLogStream();
                }
                startPollingJobStatus(); // Ensure polling is active if job is running
            } else {
                // If job is completed, failed, or idle, ensure polling is stopped.
                stopPollingJobStatus();
                if (eventSource && data.status !== 'running') { // If status is not running, close SSE
                    eventSource.close();
                    eventSource = null;
                }
            }
        })
        .catch(error => {
            console.error('Error fetching job status:', error);
            appendToLog('Error fetching job status.');
            updateStatusDisplay('error', '-', '-');
            // Potentially stop polling on error to prevent spamming
            // stopPollingJobStatus(); 
        });
}


function startPollingJobStatus() {
    if (isPolling) return; // Prevent multiple polling loops

    isPolling = true;
    startTrainButton.disabled = true; // Disable button while polling a running job
    
    async function poll() {
        if (!isPolling) return; // Stop if flag is set to false

        try {
            const response = await fetch(`${API_BASE_URL}/training_status`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            
            updateStatusDisplay(data.status, data.best_pt_path, data.converted_model_path);

            if (data.status === 'running') {
                setTimeout(poll, 3000); // Poll again after 3 seconds
            } else {
                // Job finished (completed, failed, idle)
                isPolling = false;
                startTrainButton.disabled = false;
                if (eventSource) {
                    eventSource.close(); // Ensure SSE is closed if job ended
                    eventSource = null;
                }
                appendToLog(`Job finished with status: ${data.status}. Polling stopped.`);
            }
        } catch (error) {
            console.error('Error polling job status:', error);
            appendToLog('Error polling job status. Polling stopped.');
            isPolling = false; // Stop polling on error
            startTrainButton.disabled = false;
            updateStatusDisplay('error (polling)', '-', '-');
        }
    }
    poll(); // Start the first poll
}

function stopPollingJobStatus() {
    if (isPolling) {
        isPolling = false;
        // No interval ID to clear with this async/await setTimeout pattern,
        // the `isPolling` flag handles termination.
        appendToLog('Polling stopped.');
    }
}
