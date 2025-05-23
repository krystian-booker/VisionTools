import React from 'react';
import './Sidebar.css'; // We'll create this later

function Sidebar({ onViewChange }) {
  return (
    <div className="App-sidebar">
      <ul>
        <li onClick={() => onViewChange('dashboard')}>Dashboard</li>
        <li onClick={() => onViewChange('config')}>Camera Config</li>
      </ul>
    </div>
  );
}

export default Sidebar;
