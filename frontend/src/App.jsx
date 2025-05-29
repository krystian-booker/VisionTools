import React, { useState } from 'react';
import Box from '@mui/material/Box';
import Sidebar from './components/Sidebar.jsx';
import Dashboard from './screens/Dashboard.jsx';
import CameraConfig from './screens/CameraConfig.jsx';

function App() {
  const [currentView, setCurrentView] = useState('dashboard'); // 'dashboard' or 'config'

  const handleViewChange = (view) => {
    setCurrentView(view);
  };

  return (
    <Box sx={{ display: 'flex' }}>
      <Sidebar onViewChange={handleViewChange} />
      <Box component="main" sx={{ flexGrow: 1, p: 3 }}>
        {currentView === 'dashboard' && <Dashboard />}
        {currentView === 'config' && <CameraConfig />}
      </Box>
    </Box>
  );
}

export default App;
