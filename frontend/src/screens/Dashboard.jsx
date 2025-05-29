import React, { useState, useEffect, useCallback } from 'react';
import Button from '@mui/material/Button';
import Typography from '@mui/material/Typography';
import CircularProgress from '@mui/material/CircularProgress';
import Alert from '@mui/material/Alert';
import Box from '@mui/material/Box';
import Paper from '@mui/material/Paper';

function Dashboard() {
  const [isConnected, setIsConnected] = useState(null); // null, true, or false
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const fetchStatus = useCallback(async () => {
    setIsLoading(true);
    setError('');
    setIsConnected(null); // Reset status while loading
    try {
      const response = await fetch('http://localhost:8000/status');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setIsConnected(data.connected);
    } catch (e) {
      console.error("Failed to fetch ROS status:", e);
      setError(`Failed to fetch status: ${e.message}`);
      setIsConnected(false); // Assume not connected on error
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchStatus();
  }, [fetchStatus]);

  return (
    <Paper sx={{ p: 3 }}>
      <Typography variant="h4" component="h1" gutterBottom>
        Dashboard
      </Typography>
      <Button variant="contained" onClick={fetchStatus} sx={{ mb: 2 }}>
        Refresh Status
      </Button>

      {isLoading && (
        <Box sx={{ display: 'flex', justifyContent: 'center', my: 2 }}>
          <CircularProgress />
        </Box>
      )}
      {error && !isLoading && (
        <Alert severity="error" sx={{ my: 2 }}>
          {error}
        </Alert>
      )}
      {!isLoading && !error && isConnected === true && (
        <Alert severity="success" sx={{ my: 2 }}>
          Connected to ROS
        </Alert>
      )}
      {!isLoading && !error && isConnected === false && (
        <Alert severity="warning" sx={{ my: 2 }}>
          Not connected to ROS
        </Alert>
      )}
    </Paper>
  );
}

export default Dashboard;
