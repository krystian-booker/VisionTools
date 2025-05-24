// frontend/src/components/training/LogDisplay.js
import React, { useEffect, useRef } from 'react';
import { useSelector } from 'react-redux';
import { Paper, Typography, Box } from '@mui/material';

const LogDisplay = () => {
    const logs = useSelector((state) => state.training.logs);
    const logContainerRef = useRef(null);

    useEffect(() => {
        // Auto-scroll to bottom
        if (logContainerRef.current) {
            logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
        }
    }, [logs]); // Dependency array includes logs, so it runs when logs change

    return (
        <Paper elevation={3} sx={{ p: 2, mt: 2 }}>
            <Typography variant="h6" gutterBottom>
                Console Logs
            </Typography>
            <Box
                ref={logContainerRef}
                sx={{
                    height: '400px', // Or any desired height
                    overflowY: 'scroll',
                    backgroundColor: 'grey.900', // Dark background for console look
                    color: 'common.white',
                    fontFamily: 'monospace',
                    fontSize: '0.875rem',
                    p: 2, // Padding inside the log box
                    whiteSpace: 'pre-wrap', // Preserve whitespace and newlines
                    wordBreak: 'break-all', // Break long words/lines
                    borderRadius: 1, // Optional: match Paper's border radius
                }}
            >
                {logs.length === 0 ? (
                    <Typography sx={{ color: 'grey.500' }}>Waiting for logs...</Typography>
                ) : (
                    logs.map((log, index) => (
                        // Using a div or Typography per line; Typography might add margins
                        // Using fragment and text directly for simpler rendering of pre-formatted lines
                        <React.Fragment key={index}> 
                            {log}
                            <br />
                        </React.Fragment>
                    ))
                )}
            </Box>
        </Paper>
    );
};

export default LogDisplay;
