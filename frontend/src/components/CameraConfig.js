import React, { useState, useEffect, useCallback } from 'react';
import './CameraConfig.css';

const defaultConfig = {
  serial: '',
  topic: '',
  fps: 30,
  use_auto_exposure: true,
  exposure_time: 10000,
  gain: 0,
  // Add any other default fields your backend Camera model might have
};

function CameraConfig() {
  const [configs, setConfigs] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [saveMessage, setSaveMessage] = useState('');

  const fetchConfig = useCallback(async () => {
    setIsLoading(true);
    setError('');
    try {
      const response = await fetch('http://localhost:8000/config');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setConfigs(data.length > 0 ? data : []); // Ensure it's an array
    } catch (e) {
      console.error("Failed to fetch config:", e);
      setError(`Failed to fetch config: ${e.message}`);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchConfig();
  }, [fetchConfig]);

  const handleInputChange = (index, event) => {
    const { name, value, type, checked } = event.target;
    const newConfigs = [...configs];
    newConfigs[index] = {
      ...newConfigs[index],
      [name]: type === 'checkbox' ? checked : (type === 'number' ? parseInt(value, 10) : value),
    };
    setConfigs(newConfigs);
  };

  const handleAddCamera = () => {
    setConfigs([...configs, { ...defaultConfig, serial: `camera_${configs.length + 1}` }]);
  };

  const handleRemoveCamera = (index) => {
    const newConfigs = configs.filter((_, i) => i !== index);
    setConfigs(newConfigs);
  };

  const handleSaveConfig = async () => {
    setIsLoading(true);
    setError('');
    setSaveMessage('');
    try {
      const response = await fetch('http://localhost:8000/config', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(configs),
      });
      if (!response.ok) {
        const errorData = await response.text(); // Or response.json() if backend sends structured error
        throw new Error(`HTTP error! status: ${response.status} - ${errorData}`);
      }
      const result = await response.json();
      setSaveMessage(result.message || 'Configuration saved successfully!');
      // Optionally re-fetch config to ensure sync, though backend should be source of truth
      // fetchConfig(); 
    } catch (e) {
      console.error("Failed to save config:", e);
      setError(`Failed to save config: ${e.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  if (isLoading && configs.length === 0) { // Show loading only on initial load
    return <p>Loading configuration...</p>;
  }

  return (
    <div className="camera-config-container">
      <h2>Camera Configuration</h2>
      {error && <p style={{ color: 'red' }}>Error: {error}</p>}
      {saveMessage && <p style={{ color: 'green' }}>{saveMessage}</p>}

      {configs.map((camera, index) => (
        <div key={index} className="camera-entry">
          <h3>Camera {index + 1} (Serial: {camera.serial || 'New'})</h3>
          <label htmlFor={`serial-${index}`}>Serial:</label>
          <input
            type="text"
            id={`serial-${index}`}
            name="serial"
            value={camera.serial || ''}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`topic-${index}`}>Topic:</label>
          <input
            type="text"
            id={`topic-${index}`}
            name="topic"
            value={camera.topic || ''}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`fps-${index}`}>FPS:</label>
          <input
            type="number"
            id={`fps-${index}`}
            name="fps"
            value={camera.fps || 0}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`use_auto_exposure-${index}`}>
            Use Auto Exposure:
            <input
              type="checkbox"
              id={`use_auto_exposure-${index}`}
              name="use_auto_exposure"
              checked={camera.use_auto_exposure || false}
              onChange={(e) => handleInputChange(index, e)}
            />
          </label>
          
          <label htmlFor={`exposure_time-${index}`}>Exposure Time (µs):</label>
          <input
            type="number"
            id={`exposure_time-${index}`}
            name="exposure_time"
            value={camera.exposure_time || 0}
            onChange={(e) => handleInputChange(index, e)}
            disabled={camera.use_auto_exposure}
          />

          <label htmlFor={`gain-${index}`}>Gain:</label>
          <input
            type="number"
            id={`gain-${index}`}
            name="gain"
            value={camera.gain || 0}
            onChange={(e) => handleInputChange(index, e)}
            disabled={camera.use_auto_exposure}
          />
          
          {/* Add other fields from your Pydantic model as needed */}

          <button onClick={() => handleRemoveCamera(index)}>Remove Camera</button>
        </div>
      ))}
      
      <div className="camera-config-actions">
        <button onClick={handleAddCamera}>Add Camera</button>
        <button onClick={handleSaveConfig} disabled={isLoading}>
          {isLoading ? 'Saving...' : 'Save Configuration'}
        </button>
      </div>
    </div>
  );
}

export default CameraConfig;
