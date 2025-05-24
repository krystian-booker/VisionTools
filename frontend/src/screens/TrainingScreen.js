// frontend/src/screens/TrainingScreen.js
import React, { useEffect, useRef, useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import {
    fetchTrainingConfigOptions,
    fetchTrainingJobStatus,
    addLogLine,
    setSseConnected,
    resetTrainingState, // Optional: for a manual reset button
    setJobTerminated,   // To handle SSE termination event
    clearLogs
} from '../features/training/trainingSlice';

import TrainingForm from '../components/training/TrainingForm';
import LogDisplay from '../components/training/LogDisplay';
import JobStatusDisplay from '../components/training/JobStatusDisplay';

import { Container, Typography, Box, Button, Alert } from '@mui/material';

const API_BASE_URL = 'http://localhost:5000'; // Should match slice

const TrainingScreen = () => {
    const dispatch = useDispatch();
    const { jobStatus, sseConnected, error: globalTrainingError } = useSelector((state) => state.training);
    const eventSourceRef = useRef(null);
    const pollingIntervalRef = useRef(null);

    // Fetch initial config and status
    useEffect(() => {
        dispatch(fetchTrainingConfigOptions());
        dispatch(fetchTrainingJobStatus()); // Fetch current status on load
    }, [dispatch]);

    // Memoized function to handle SSE setup
    const setupEventSource = useCallback(() => {
        if (eventSourceRef.current) {
            eventSourceRef.current.close();
        }
        
        const es = new EventSource(`${API_BASE_URL}/training_log_stream`);
        eventSourceRef.current = es;
        dispatch(setSseConnected(true));
        dispatch(addLogLine('SSE: Log stream connected.'));

        es.onmessage = (event) => {
            dispatch(addLogLine(event.data));
        };

        es.addEventListener('job_terminated', (event) => {
            const finalStatus = event.data; // 'completed' or 'failed'
            dispatch(addLogLine(`SSE: Job terminated event received. Final status: ${finalStatus}`));
            dispatch(setSseConnected(false));
            // Trigger a final status fetch to get all details (paths, errors)
            dispatch(fetchTrainingJobStatus()).then(() => {
                 // Optionally, use setJobTerminated if you want to update state directly from SSE
                 // For now, relying on fetchTrainingJobStatus to be the source of truth for final state.
                 // dispatch(setJobTerminated({ finalStatus, ...parsedDataFromEventIfNeeded }));
            });
            if (eventSourceRef.current) {
                eventSourceRef.current.close();
                eventSourceRef.current = null;
            }
        });

        es.onerror = (err) => {
            dispatch(addLogLine('SSE: Log stream error or closed.'));
            console.error('EventSource error:', err);
            dispatch(setSseConnected(false));
            // Don't close here if it's a temporary network error and job might still be running.
            // Polling will eventually update status. If job truly ended, 'job_terminated' should have fired.
            // If jobStatus is still 'running', and error persists, it might indicate a backend issue.
            if (jobStatus !== 'running' && eventSourceRef.current) { // Only close if job is not supposed to be running
                eventSourceRef.current.close();
                eventSourceRef.current = null;
            }
        };
    }, [dispatch, jobStatus]); // jobStatus in dependencies to re-evaluate if needed, though setup is mostly on 'running'

    // Manage SSE connection and Polling based on jobStatus
    useEffect(() => {
        if (jobStatus === 'running') {
            if (!sseConnected) { // Check Redux state for sseConnected
                setupEventSource();
            }
            // Start polling
            if (!pollingIntervalRef.current) {
                pollingIntervalRef.current = setInterval(() => {
                    dispatch(fetchTrainingJobStatus());
                }, 5000); // Poll every 5 seconds
            }
        } else if (jobStatus === 'initiating') {
            // Waiting for job to become 'running', SSE and polling will start then.
            // Could have a timeout here to revert to 'failed' if it stays 'initiating' too long.
        } else { // 'idle', 'completed', 'failed', 'error'
            if (eventSourceRef.current) {
                eventSourceRef.current.close();
                eventSourceRef.current = null;
                dispatch(setSseConnected(false));
                dispatch(addLogLine('SSE: Log stream disconnected (job not running).'));
            }
            if (pollingIntervalRef.current) {
                clearInterval(pollingIntervalRef.current);
                pollingIntervalRef.current = null;
            }
        }

        // Cleanup on component unmount
        return () => {
            if (eventSourceRef.current) {
                eventSourceRef.current.close();
                dispatch(setSseConnected(false));
            }
            if (pollingIntervalRef.current) {
                clearInterval(pollingIntervalRef.current);
            }
        };
    }, [jobStatus, sseConnected, dispatch, setupEventSource]);
    
    const handleManualReset = () => {
        dispatch(resetTrainingState());
        dispatch(clearLogs()); // Ensure logs are visually cleared immediately
        // Optionally re-fetch config if it could change or if form should reset to initial defaults
        // dispatch(fetchTrainingConfigOptions()); 
    };

    return (
        <Container maxWidth="lg" sx={{ mt: 2, mb: 4 }}>
            <Typography variant="h4" component="h1" gutterBottom>
                YOLOv5 Model Training & Conversion
            </Typography>
            
            {/* Global error display for issues not specific to a component like config load failure */}
            {globalTrainingError && (jobStatus === 'failed' || jobStatus === 'error') && (
                 <Alert severity="error" sx={{ mb: 2 }}>
                    Global Error: {typeof globalTrainingError === 'object' ? JSON.stringify(globalTrainingError) : globalTrainingError}
                 </Alert>
            )}

            <TrainingForm />
            <JobStatusDisplay />
            <LogDisplay />

            <Box sx={{ mt: 2, display: 'flex', justifyContent: 'flex-end' }}>
                <Button 
                    variant="outlined" 
                    color="warning" 
                    onClick={handleManualReset}
                    disabled={jobStatus === 'running' || jobStatus === 'initiating'}
                >
                    Reset Training State
                </Button>
            </Box>
        </Container>
    );
};

export default TrainingScreen;
