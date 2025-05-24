import React, { useState, useEffect, useCallback } from 'react';
import './Dashboard.css'; // Import the new CSS file

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
    <div className="dashboard-container"> {/* Added container class */}
      <h2>Dashboard</h2>
      <button onClick={fetchStatus}>Refresh Status</button>
      {error && <p className="error-message">{error}</p>} {/* Added error class */}
      {rosStatus === null && <p className="status-loading">Loading status...</p>} {/* Added loading class */}
      {rosStatus === true && <p className="status-connected">Connected to ROS</p>} {/* Added connected class */}
      {rosStatus === false && !error && <p className="status-not-connected">Not connected to ROS</p>} {/* Added not-connected class */}
    </div>
  );
}

export default Dashboard;
