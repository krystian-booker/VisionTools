// frontend/src/features/training/trainingSlice.js
import { createSlice, createAsyncThunk } from '@reduxjs/toolkit';
import axios from 'axios';

const API_BASE_URL = 'http://localhost:5000'; // Adjust if necessary

const initialFormParameters = {
    data_yaml_path: '',
    model_type: '', // Will be set from config defaults
    epochs: 100,    // Default, will be updated from config
    imgsz: 416,     // Default, will be updated from config
    device: 'cpu',  // Default, will be updated from config
    conversion_target: '', // Will be set from config defaults (or allow none)
    run_name: '',
};

const initialState = {
    formParameters: { ...initialFormParameters },
    jobStatus: 'idle', // 'idle' | 'initiating' | 'running' | 'completed' | 'failed' | 'error'
    logs: [], // Array of log lines
    bestModelPath: null,
    convertedModelPath: null,
    error: null, // For API errors or job failures
    sseConnected: false,
    configOptions: {
        model_types: [],
        conversion_targets: [],
        default_training_parameters: {},
    },
    configStatus: 'idle', // 'idle' | 'loading' | 'succeeded' | 'failed'
    currentJobId: null, // Could be PID from backend if useful for tracking a specific job instance
};

// Async Thunks
export const fetchTrainingConfigOptions = createAsyncThunk(
    'training/fetchConfigOptions',
    async (_, { rejectWithValue }) => {
        try {
            const response = await axios.get(`${API_BASE_URL}/config_options`);
            return response.data;
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

export const startTrainingJob = createAsyncThunk(
    'training/startTrainingJob',
    async (formParameters, { rejectWithValue }) => {
        try {
            const response = await axios.post(`${API_BASE_URL}/train_and_convert`, formParameters);
            // Expected backend response on success (202):
            // { status: 'started', message: 'Training pipeline initiated.', job_pid: ..., log_file: ..., job_details: { status: 'running', ... } }
            return response.data;
        } catch (error) {
            // error.response.data might be: { status: 'error', message: 'A job is already running...' }
            return rejectWithValue(error.response?.data || { message: error.message });
        }
    }
);

export const fetchTrainingJobStatus = createAsyncThunk(
    'training/fetchJobStatus',
    async (_, { rejectWithValue }) => { // No specific args needed, uses global job state on backend
        try {
            const response = await axios.get(`${API_BASE_URL}/training_status`);
            // Expected backend response:
            // { status: 'running'/'completed'/'failed'/'idle', pid: ..., best_pt_path: ..., converted_model_path: ..., error_message: ... }
            return response.data;
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

const trainingSlice = createSlice({
    name: 'training',
    initialState,
    reducers: {
        setFormField: (state, action) => {
            const { field, value } = action.payload;
            state.formParameters[field] = value;
        },
        addLogLine: (state, action) => {
            state.logs.push(action.payload);
        },
        clearLogs: (state) => {
            state.logs = [];
        },
        setSseConnected: (state, action) => {
            state.sseConnected = action.payload;
        },
        resetTrainingState: (state) => {
            state.jobStatus = 'idle';
            state.logs = [];
            state.bestModelPath = null;
            state.convertedModelPath = null;
            state.error = null;
            state.sseConnected = false;
            state.currentJobId = null;
            // Keep configOptions and formParameters (as they might have user edits or defaults)
            // Or reset formParameters to defaults from config:
            // state.formParameters = {
            //     ...initialFormParameters,
            //     ...state.configOptions.default_training_parameters,
            //     model_type: state.configOptions.model_types[0] || '',
            //     // conversion_target: can be kept or reset
            // };
        },
        // Used to update status from SSE 'job_terminated' event if needed
        setJobTerminated: (state, action) => {
            const { finalStatus, bestPath, convertedPath, errorMessage } = action.payload;
            state.jobStatus = finalStatus;
            state.sseConnected = false; // SSE connection should close
            if (finalStatus === 'completed') {
                state.bestModelPath = bestPath;
                state.convertedModelPath = convertedPath;
                state.error = null;
            } else if (finalStatus === 'failed') {
                state.error = errorMessage || 'Job failed (reported by SSE).';
            }
            state.currentJobId = null;
        }
    },
    extraReducers: (builder) => {
        builder
            // fetchTrainingConfigOptions
            .addCase(fetchTrainingConfigOptions.pending, (state) => {
                state.configStatus = 'loading';
            })
            .addCase(fetchTrainingConfigOptions.fulfilled, (state, action) => {
                state.configStatus = 'succeeded';
                state.configOptions = action.payload;
                // Set default form parameters from fetched config
                state.formParameters.model_type = action.payload.model_types?.[0] || '';
                // Ensure conversion_target can be empty string if no default or "None" option preferred
                state.formParameters.conversion_target = action.payload.conversion_targets?.[0] || ''; 
                if (action.payload.default_training_parameters) {
                    state.formParameters.epochs = action.payload.default_training_parameters.epochs || state.formParameters.epochs;
                    state.formParameters.imgsz = action.payload.default_training_parameters.imgsz || state.formParameters.imgsz;
                    state.formParameters.device = action.payload.default_training_parameters.device || state.formParameters.device;
                }
            })
            .addCase(fetchTrainingConfigOptions.rejected, (state, action) => {
                state.configStatus = 'failed';
                state.error = action.payload || action.error.message; // Store error related to config fetching
            })

            // startTrainingJob
            .addCase(startTrainingJob.pending, (state) => {
                state.jobStatus = 'initiating';
                state.logs = []; // Clear logs for new job
                state.bestModelPath = null;
                state.convertedModelPath = null;
                state.error = null;
                state.currentJobId = null;
            })
            .addCase(startTrainingJob.fulfilled, (state, action) => {
                // Backend response: { status: 'started', job_pid: ..., job_details: { status: 'running', ... } }
                if (action.payload.status === 'started' && action.payload.job_details) {
                    state.jobStatus = action.payload.job_details.status; // Should be 'running'
                    state.currentJobId = action.payload.job_pid;
                    state.error = null;
                } else {
                    // This case should ideally be handled by rejectWithValue in the thunk for non-2xx responses
                    // or if backend returns 2xx but with an error status internally (e.g. job already running)
                    state.jobStatus = 'error'; // Or 'failed'
                    state.error = action.payload.message || 'Failed to start job: unexpected backend response.';
                }
            })
            .addCase(startTrainingJob.rejected, (state, action) => {
                state.jobStatus = 'failed';
                state.error = action.payload?.message || action.error?.message || 'Failed to start training job.';
            })

            // fetchTrainingJobStatus
            .addCase(fetchTrainingJobStatus.pending, (state) => {
                // Optionally indicate that status is being refreshed, e.g., state.statusRefresh = 'loading'
                // For now, no explicit visual feedback for pending status fetch, it's usually quick.
            })
            .addCase(fetchTrainingJobStatus.fulfilled, (state, action) => {
                const { status, best_pt_path, converted_model_path, error_message, pid } = action.payload;
                state.jobStatus = status;
                state.bestModelPath = best_pt_path;
                state.convertedModelPath = converted_model_path;
                state.currentJobId = pid; // Update PID, could be null if not running

                if (status === 'failed' || status === 'error') {
                    state.error = error_message || 'Job reported failure or error.';
                } else {
                    state.error = null; // Clear error if job is running or completed successfully
                }
                // If status is completed/failed/idle, SSE should be disconnected by TrainingScreen
                if (status !== 'running' && status !== 'initiating') {
                    state.sseConnected = false;
                }
            })
            .addCase(fetchTrainingJobStatus.rejected, (state, action) => {
                // Don't necessarily change jobStatus here, as it might be a temporary API glitch
                // But do record the error.
                state.error = `Failed to fetch job status: ${action.payload || action.error.message}`;
                // Could set a specific flag like state.statusRefreshError = true;
            });
    },
});

export const { 
    setFormField, 
    addLogLine, 
    clearLogs, 
    setSseConnected, 
    resetTrainingState,
    setJobTerminated 
} = trainingSlice.actions;

export default trainingSlice.reducer;
