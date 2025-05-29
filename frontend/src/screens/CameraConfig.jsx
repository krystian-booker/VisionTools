import React, { useState, useEffect, useCallback } from 'react';
import {
  Button, Typography, Box, CircularProgress, Alert
} from '@mui/material';
import AddIcon from '@mui/icons-material/Add';
import CameraCard from '../components/cameraConfig/CameraCard.jsx';

const defaultConfig = {
  name: 'New Camera',
  serial: '', 
  topic: '',
  debug: false,
  compute_brightness: true,
  dump_node_map: false,
  gain_auto: 'Off',
  exposure_auto: 'Off',
  offset_x: 0,
  offset_y: 0,
  image_width: 2048,
  image_height: 1536,
  pixel_format: 'Mono8',
  frame_rate_continuous: true,
  frame_rate: 100.0,
  trigger_mode: 'On',
  chunk_mode_active: true,
  chunk_selector_frame_id: 'FrameID',
  chunk_enable_frame_id: true,
  chunk_selector_exposure_time: 'ExposureTime',
  chunk_enable_exposure_time: true,
  chunk_selector_gain: 'Gain',
  chunk_enable_gain: true,
  chunk_selector_timestamp: 'Timestamp',
  chunk_enable_timestamp: true,
  adjust_timestamp: true,
  gain: 0,
  exposure_time: 9000,
  line2_selector: 'Line2',
  line2_v33enable: false,
  line3_selector: 'Line3',
  line3_linemode: 'Input',
  trigger_selector: 'FrameStart',
  trigger_source: 'Line3',
  trigger_delay: 9,
  trigger_overlap: 'ReadOut',
};

function CameraConfig() {
  const [configs, setConfigs] = useState([]);
  const [isLoading, setIsLoading] = useState(false); // For initial fetch and save
  const [isFetching, setIsFetching] = useState(true); // Specifically for initial fetch
  const [error, setError] = useState('');
  const [saveMessage, setSaveMessage] = useState('');

  const fetchConfig = useCallback(async () => {
    setIsFetching(true);
    setError('');
    try {
      const response = await fetch('http://localhost:8000/config');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setConfigs(data.length > 0 ? data : []);
    } catch (e) {
      console.error("Failed to fetch config:", e);
      setError(`Failed to fetch config: ${e.message}`);
    } finally {
      setIsFetching(false);
    }
  }, []);

  useEffect(() => {
    fetchConfig();
  }, [fetchConfig]);

  const handleInputChange = (index, event) => {
    const { name, value, type, checked } = event.target;
    const newConfigs = [...configs];
    let val;

    if (name === 'pixel_format') { // Special handling for Select
        val = value;
    } else if (type === 'checkbox') {
      val = checked;
    } else if (type === 'number') {
      val = name === 'frame_rate' ? parseFloat(value) : parseInt(value, 10);
      if (isNaN(val)) {
        val = name === 'frame_rate' ? 0.0 : 0;
      }
    } else {
      val = value;
    }
    
    newConfigs[index] = {
      ...newConfigs[index],
      [name]: val,
    };
    setConfigs(newConfigs);
  };

  const handleAddCamera = () => {
    setConfigs([...configs, { ...defaultConfig, name: `Camera ${configs.length + 1}` }]);
  };

  const handleRemoveCamera = (index) => {
    const newConfigs = configs.filter((_, i) => i !== index);
    setConfigs(newConfigs);
  };

  const handleSaveConfig = async () => {
    setIsLoading(true); // Use general isLoading for save operation
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
        const errorData = await response.text();
        throw new Error(`HTTP error! status: ${response.status} - ${errorData}`);
      }
      const result = await response.json();
      setSaveMessage(result.message || 'Configuration saved successfully!');
      // fetchConfig(); // Optionally re-fetch
    } catch (e) {
      console.error("Failed to save config:", e);
      setError(`Failed to save config: ${e.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  if (isFetching) {
    return (
      <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '80vh' }}>
        <CircularProgress />
      </Box>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h4" component="h1" gutterBottom>
        Camera Configuration
      </Typography>

      {error && <Alert severity="error" sx={{ mb: 2 }}>{error}</Alert>}
      {saveMessage && <Alert severity="success" sx={{ mb: 2 }}>{saveMessage}</Alert>}

      {configs.map((camera, index) => (
        <CameraCard
          key={index} // Using index as key for simplicity, consider camera.serial if available and unique
          camera={camera}
          index={index}
          handleInputChange={handleInputChange}
          handleRemoveCamera={handleRemoveCamera}
        />
      ))}

      <Box sx={{ mt: 3, display: 'flex', gap: 2 }}>
        <Button variant="outlined" onClick={handleAddCamera} startIcon={<AddIcon />}>
          Add Camera
        </Button>
        <Button variant="contained" color="primary" onClick={handleSaveConfig} disabled={isLoading}>
          {isLoading ? <CircularProgress size={24} /> : 'Save Configuration'}
        </Button>
      </Box>
    </Box>
  );
}

export default CameraConfig;
