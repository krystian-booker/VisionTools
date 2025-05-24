import { createSlice, createAsyncThunk } from '@reduxjs/toolkit';
import axios from 'axios';

const API_BASE_URL = 'http://localhost:5000'; // Adjust if necessary

const initialState = {
    nodes: [],
    status: 'idle', // 'idle' | 'loading' | 'succeeded' | 'failed'
    error: null,
};

// Async Thunks
export const fetchRosNodes = createAsyncThunk('rosNodes/fetchRosNodes', async (_, { rejectWithValue }) => {
    try {
        const response = await axios.get(`${API_BASE_URL}/ros/nodes`);
        return response.data;
    } catch (error) {
        return rejectWithValue(error.response?.data?.message || error.message);
    }
});

export const toggleNodeEnabled = createAsyncThunk(
    'rosNodes/toggleNodeEnabled',
    async ({ nodeId, isEnabled }, { rejectWithValue }) => {
        try {
            const endpoint = isEnabled ? 'enable' : 'disable';
            const response = await axios.post(`${API_BASE_URL}/ros/nodes/${nodeId}/${endpoint}`);
            return response.data.node; // Assuming backend returns the updated node
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

export const startRosNode = createAsyncThunk(
    'rosNodes/startRosNode', 
    async (nodeId, { rejectWithValue }) => {
        try {
            const response = await axios.post(`${API_BASE_URL}/ros/nodes/${nodeId}/start`);
            // The backend returns: { status: 'success', message: '...', pid: ..., node_id: ..., last_started: ... }
            // or { status: 'error', message: '...', http_status_code: ...}
            if (response.data.status === 'error' || response.data.status === 'info') { // info for already running
                 // For errors or info like "already running", we might want to pass the whole payload
                 // for the reducer to decide how to update state, or just re-fetch.
                 // Here, we pass it along, and the reducer will update the node based on it.
                 // If it's an actual error from start_node, it should have an http_status_code >= 400
                 // The 'start_node' function in manager returns a dict like:
                 // {'status': 'error', 'message': f'Node with ID {node_id} not found.', 'http_status_code': 404}
                 // or {'status': 'success', ..., 'pid': pid, 'last_started': time, http_status_code: 200}
                 // The slice should handle based on 'status' field from response.data primarily.
                if(response.data.http_status_code && response.data.http_status_code >= 400) {
                    return rejectWithValue(response.data.message || 'Failed to start node.');
                }
            }
            return response.data; 
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

export const stopRosNode = createAsyncThunk(
    'rosNodes/stopRosNode', 
    async (nodeId, { rejectWithValue }) => {
        try {
            const response = await axios.post(`${API_BASE_URL}/ros/nodes/${nodeId}/stop`);
            // Backend returns: { status: 'success', message: '...', node_id: ..., last_stopped: ... }
            if (response.data.status === 'error') {
                return rejectWithValue(response.data.message || 'Failed to stop node.');
            }
            return response.data;
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

export const restartRosNode = createAsyncThunk(
    'rosNodes/restartRosNode', 
    async (nodeId, { rejectWithValue }) => {
        try {
            const response = await axios.post(`${API_BASE_URL}/ros/nodes/${nodeId}/restart`);
            // Restart calls stop then start. Response is from start_node.
            if (response.data.status === 'error') {
                return rejectWithValue(response.data.message || 'Failed to restart node.');
            }
            return response.data;
        } catch (error) {
            return rejectWithValue(error.response?.data?.message || error.message);
        }
    }
);

const rosNodesSlice = createSlice({
    name: 'rosNodes',
    initialState,
    reducers: {
        // Example: Manually update a node's status if needed from other sources
        // updateNodeStatusLocally: (state, action) => {
        //   const { nodeId, newStatus, pid } = action.payload;
        //   const existingNode = state.nodes.find(node => node.id === nodeId);
        //   if (existingNode) {
        //     existingNode.status = newStatus;
        //     if (pid !== undefined) existingNode.pid = pid;
        //   }
        // },
    },
    extraReducers: (builder) => {
        builder
            // fetchRosNodes
            .addCase(fetchRosNodes.pending, (state) => {
                state.status = 'loading';
            })
            .addCase(fetchRosNodes.fulfilled, (state, action) => {
                state.status = 'succeeded';
                state.nodes = action.payload;
                state.error = null;
            })
            .addCase(fetchRosNodes.rejected, (state, action) => {
                state.status = 'failed';
                state.error = action.payload || action.error.message;
            })

            // toggleNodeEnabled
            .addCase(toggleNodeEnabled.pending, (state, action) => {
                const { nodeId } = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'toggling'; // Temporary status
                }
            })
            .addCase(toggleNodeEnabled.fulfilled, (state, action) => {
                const updatedNode = action.payload; 
                const index = state.nodes.findIndex(node => node.id === updatedNode.id);
                if (index !== -1) {
                    state.nodes[index] = updatedNode; // Backend returns the full node, status included
                }
                state.error = null;
            })
            .addCase(toggleNodeEnabled.rejected, (state, action) => {
                const { nodeId } = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) { // Revert to original status or fetch fresh
                    // For now, just clear 'toggling'. A fetchRosNodes might be better.
                    // Or, store original status before pending and revert to it.
                    // For simplicity, we'll rely on a subsequent fetch or the error message.
                    // If the backend didn't change it, it's still the old status.
                    // If the operation failed, the backend state is the source of truth.
                    // Fetching all nodes again is safer upon critical error.
                    // For now, just marking global error.
                }
                state.status = 'failed'; // Global status
                state.error = action.payload || action.error.message;
            })

            // startRosNode
            .addCase(startRosNode.pending, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'starting';
                }
            })
            .addCase(startRosNode.fulfilled, (state, action) => {
                const { node_id, pid, last_started, status: responseStatus, message } = action.payload;
                const nodeToUpdate = state.nodes.find(node => node.id === node_id);
                if (nodeToUpdate) {
                    // Based on thunk logic, fulfilled means backend returned 200
                    // and response.data.status was 'success' or 'info' (but info with 409 is rejected).
                    // So here, responseStatus should be 'success'.
                    if (responseStatus === 'success') {
                        nodeToUpdate.status = 'running';
                        nodeToUpdate.pid = pid;
                        nodeToUpdate.last_started = last_started;
                    } else if (responseStatus === 'info' && message && message.includes('already running')) {
                        // This case might be hit if the thunk logic for info (409) changes
                        nodeToUpdate.status = 'running'; 
                        if (pid) nodeToUpdate.pid = pid;
                    } else { 
                        // Fallback if backend gives 200 but status isn't 'success' or expected 'info'
                        // This indicates an unexpected successful response from backend.
                        // For safety, reflect what backend sent if possible, or mark as 'unknown'.
                        // Or, more robustly, re-fetch the node's true status.
                        // For now, assume 'running' if fulfilled and not explicitly 'info'.
                        nodeToUpdate.status = 'running'; 
                        if(pid) nodeToUpdate.pid = pid;
                        if(last_started) nodeToUpdate.last_started = last_started;
                        logger.warn(`startRosNode.fulfilled with unexpected responseStatus: ${responseStatus}`);
                    }
                }
                state.error = null; // Clear global error on individual success
            })
            .addCase(startRosNode.rejected, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'error'; // Or revert to previous status if known
                }
                state.status = 'failed'; // Global status
                state.error = action.payload || action.error.message;
            })

            // stopRosNode
            .addCase(stopRosNode.pending, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'stopping';
                }
            })
            .addCase(stopRosNode.fulfilled, (state, action) => {
                const { node_id, last_stopped, status: responseStatus } = action.payload;
                const nodeToUpdate = state.nodes.find(node => node.id === node_id);
                if (nodeToUpdate) {
                    // Backend returns status 'success', 'info' (already stopped), or 'warning' (termination not confirmed)
                    // All these mean the node is effectively considered stopped from a control perspective.
                    nodeToUpdate.status = 'stopped'; 
                    nodeToUpdate.pid = null;
                    nodeToUpdate.last_stopped = last_stopped;
                }
                state.error = null;
            })
            .addCase(stopRosNode.rejected, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'error'; // Or revert, e.g. to 'running' if stop failed critically
                }
                state.status = 'failed';
                state.error = action.payload || action.error.message;
            })
            
            // restartRosNode
            .addCase(restartRosNode.pending, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    node.status = 'restarting';
                }
            })
            .addCase(restartRosNode.fulfilled, (state, action) => {
                const { node_id, pid, last_started, status: responseStatus, message } = action.payload;
                const nodeToUpdate = state.nodes.find(node => node.id === node_id);
                if (nodeToUpdate) {
                    // Similar to startRosNode.fulfilled, as restart's response is from the start part.
                    if (responseStatus === 'success') {
                        nodeToUpdate.status = 'running';
                        nodeToUpdate.pid = pid;
                        nodeToUpdate.last_started = last_started;
                    } else if (responseStatus === 'info' && message && message.includes('already running')) {
                        nodeToUpdate.status = 'running';
                        if (pid) nodeToUpdate.pid = pid;
                    } else {
                        nodeToUpdate.status = 'running'; // Default assumption for fulfilled
                        if(pid) nodeToUpdate.pid = pid;
                        if(last_started) nodeToUpdate.last_started = last_started;
                        logger.warn(`restartRosNode.fulfilled with unexpected responseStatus: ${responseStatus}`);
                    }
                }
                state.error = null;
            })
            .addCase(restartRosNode.rejected, (state, action) => {
                const nodeId = action.meta.arg;
                const node = state.nodes.find(n => n.id === nodeId);
                if (node) {
                    // If restart fails, node could be in various states.
                    // 'error' is a safe bet, or fetch fresh status.
                    node.status = 'error'; 
                }
                state.status = 'failed';
                state.error = action.payload || action.error.message;
            });
            // Removed the generic .addMatcher for rejected as individual rejected cases are now handled.
            // If a generic one is still desired for other thunks, it can be added back.
    },
});

// export const { updateNodeStatusLocally } = rosNodesSlice.actions; // If you have custom reducers
export default rosNodesSlice.reducer;

// Small logger for slice warnings (optional, for client-side debugging)
const logger = {
    warn: (message) => {
        if (process.env.NODE_ENV !== 'production') {
            console.warn(message);
        }
    }
};
