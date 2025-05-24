import React, { useState } from 'react';
import './App.css';
import Sidebar from './components/Sidebar';
import Dashboard from './components/Dashboard';
import CameraConfig from './components/CameraConfig';

function App() {
  const [currentView, setCurrentView] = useState('dashboard'); // 'dashboard' or 'config'

  const handleViewChange = (view) => {
    setCurrentView(view);
  };

  return (
    <div className="App">
      <Sidebar onViewChange={handleViewChange} />
      <div className="App-content">
        {currentView === 'dashboard' && <Dashboard />}
        {currentView === 'config' && <CameraConfig />}
      </div>
    </div>
  );
}

export default App;
