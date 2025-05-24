// frontend/src/components/training/JobStatusDisplay.js
import React from 'react';
import { useSelector } from 'react-redux';
import { Paper, Typography, Box, Alert, Link } from '@mui/material';

const JobStatusDisplay = () => {
    const { jobStatus, bestModelPath, convertedModelPath, error, currentJobId } = useSelector((state) => state.training);

    return (
        <Paper elevation={3} sx={{ p: 2, mt: 2 }}>
            <Typography variant="h6" gutterBottom>
                Job Status
            </Typography>
            <Box sx={{ mb: 1 }}>
                <Typography component="span" variant="body1">
                    <strong>Status: </strong>
                </Typography>
                <Typography 
                    component="span" 
                    variant="body1"
                    color={
                        jobStatus === 'completed' ? 'success.main' :
                        jobStatus === 'failed' || jobStatus === 'error' ? 'error.main' :
                        jobStatus === 'running' || jobStatus ===
                         'initiating' ? 'info.main' :
                        'text.primary'
                    }
                    sx={{ fontWeight: 'bold' }}
                >
                    {jobStatus.toUpperCase()}
                </Typography>
                {currentJobId && (jobStatus === 'running' || jobStatus === 'initiating') && (
                     <Typography component="span" variant="body2" sx={{ ml: 1 }}>(PID: {currentJobId})</Typography>
                )}
            </Box>

            {bestModelPath && (
                <Typography variant="body2">
                    <strong>Best Model (.pt): </strong> 
                    <Link href={`file://${bestModelPath}`} target="_blank" rel="noopener noreferrer">{bestModelPath}</Link>
                </Typography>
            )}

            {convertedModelPath && convertedModelPath !== "None" && (
                <Typography variant="body2">
                    <strong>Converted Model: </strong> 
                    <Link href={`file://${convertedModelPath}`} target="_blank" rel="noopener noreferrer">{convertedModelPath}</Link>
                </Typography>
            )}
            
            {error && (
                <Alert severity="error" sx={{ mt: 2 }}>
                    {typeof error === 'object' ? JSON.stringify(error) : error}
                </Alert>
            )}
        </Paper>
    );
};

export default JobStatusDisplay;
