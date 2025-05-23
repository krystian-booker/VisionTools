import React, { useState, useEffect, useCallback } from 'react';

function Dashboard() {
  const [rosStatus, setRosStatus] = useState(null); // null, true, or false
  const [error, setError] = useState('');

  const fetchStatus = useCallback(async () => {
    setError('');
    try {
      const response = await fetch('http://localhost:8000/status');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setRosStatus(data.connected);
    } catch (e) {
      console.error("Failed to fetch ROS status:", e);
      setError(`Failed to fetch status: ${e.message}`);
      setRosStatus(false); // Assume not connected on error
    }
  }, []);

  useEffect(() => {
    fetchStatus();
  }, [fetchStatus]);

  return (
    <div>
      <h2>Dashboard</h2>
      <button onClick={fetchStatus}>Refresh Status</button>
      {error && <p style={{ color: 'red' }}>{error}</p>}
      {rosStatus === null && <p>Loading status...</p>}
      {rosStatus === true && <p style={{ color: 'green' }}>Connected to ROS</p>}
      {rosStatus === false && !error && <p style={{ color: 'orange' }}>Not connected to ROS</p>}
    </div>
  );
}

export default Dashboard;
