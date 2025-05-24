import React, { useState, useEffect, useCallback } from 'react';
import './CameraConfig.css';

const defaultConfig = {
  name: 'New Camera',
  serial: '', // Should be present and empty by default.
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
    let val;
    if (type === 'checkbox') {
      val = checked;
    } else if (type === 'number') {
      val = name === 'frame_rate' ? parseFloat(value) : parseInt(value, 10);
      if (isNaN(val)) { // Handle cases where parsing might result in NaN, e.g., empty input
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
          <h3>{camera.name || `Camera ${index + 1}`}</h3>
          
          <label htmlFor={`name-${index}`}>Camera Name:</label>
          <input
            type="text"
            id={`name-${index}`}
            name="name"
            value={camera.name || ''}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`serial-${index}`}>Serial (Mandatory):</label>
          <input
            type="text"
            id={`serial-${index}`}
            name="serial"
            value={camera.serial || ''}
            onChange={(e) => handleInputChange(index, e)}
            required 
          />

          <label htmlFor={`topic-${index}`}>Topic:</label>
          <input
            type="text"
            id={`topic-${index}`}
            name="topic"
            value={camera.topic || ''}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`debug-${index}`}>Debug Mode:</label>
          <input
            type="checkbox"
            id={`debug-${index}`}
            name="debug"
            checked={camera.debug || false}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`compute_brightness-${index}`}>Compute Brightness:</label>
          <input
            type="checkbox"
            id={`compute_brightness-${index}`}
            name="compute_brightness"
            checked={camera.compute_brightness || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`dump_node_map-${index}`}>Dump Node Map:</label>
          <input
            type="checkbox"
            id={`dump_node_map-${index}`}
            name="dump_node_map"
            checked={camera.dump_node_map || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`gain_auto-${index}`}>Gain Auto (Off, Once, Continuous):</label>
          <input 
            type="text" 
            id={`gain_auto-${index}`}
            name="gain_auto"
            value={camera.gain_auto || 'Off'}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`exposure_auto-${index}`}>Exposure Auto (Off, Once, Continuous):</label>
          <input 
            type="text" 
            id={`exposure_auto-${index}`}
            name="exposure_auto"
            value={camera.exposure_auto || 'Off'}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`offset_x-${index}`}>Offset X:</label>
          <input
            type="number"
            id={`offset_x-${index}`}
            name="offset_x"
            value={camera.offset_x || 0}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`offset_y-${index}`}>Offset Y:</label>
          <input
            type="number"
            id={`offset_y-${index}`}
            name="offset_y"
            value={camera.offset_y || 0}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`image_width-${index}`}>Image Width:</label>
          <input
            type="number"
            id={`image_width-${index}`}
            name="image_width"
            value={camera.image_width || 0}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`image_height-${index}`}>Image Height:</label>
          <input
            type="number"
            id={`image_height-${index}`}
            name="image_height"
            value={camera.image_height || 0}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`pixel_format-${index}`}>Pixel Format:</label>
          <select
            id={`pixel_format-${index}`}
            name="pixel_format"
            value={camera.pixel_format || 'Mono8'}
            onChange={(e) => handleInputChange(index, e)}
          >
            <option value="Mono8">Mono8</option>
            <option value="BayerRG8">BayerRG8</option>
            <option value="RGB8">RGB8</option>
            {/* Add other formats if needed */}
          </select>

          <label htmlFor={`frame_rate_continuous-${index}`}>Frame Rate Continuous:</label>
          <input
            type="checkbox"
            id={`frame_rate_continuous-${index}`}
            name="frame_rate_continuous"
            checked={camera.frame_rate_continuous || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`frame_rate-${index}`}>Frame Rate (fps):</label>
          <input
            type="number"
            step="0.1"
            id={`frame_rate-${index}`}
            name="frame_rate"
            value={camera.frame_rate || 0}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`trigger_mode-${index}`}>Trigger Mode (On, Off):</label>
          <input
            type="text"
            id={`trigger_mode-${index}`}
            name="trigger_mode"
            value={camera.trigger_mode || 'Off'}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`chunk_mode_active-${index}`}>Chunk Mode Active:</label>
          <input
            type="checkbox"
            id={`chunk_mode_active-${index}`}
            name="chunk_mode_active"
            checked={camera.chunk_mode_active || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`chunk_selector_frame_id-${index}`}>Chunk Selector: Frame ID:</label>
          <input
            type="text"
            id={`chunk_selector_frame_id-${index}`}
            name="chunk_selector_frame_id"
            value={camera.chunk_selector_frame_id || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`chunk_enable_frame_id-${index}`}>Enable Chunk: Frame ID:</label>
          <input
            type="checkbox"
            id={`chunk_enable_frame_id-${index}`}
            name="chunk_enable_frame_id"
            checked={camera.chunk_enable_frame_id || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`chunk_selector_exposure_time-${index}`}>Chunk Selector: Exposure Time:</label>
          <input
            type="text"
            id={`chunk_selector_exposure_time-${index}`}
            name="chunk_selector_exposure_time"
            value={camera.chunk_selector_exposure_time || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`chunk_enable_exposure_time-${index}`}>Enable Chunk: Exposure Time:</label>
          <input
            type="checkbox"
            id={`chunk_enable_exposure_time-${index}`}
            name="chunk_enable_exposure_time"
            checked={camera.chunk_enable_exposure_time || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`chunk_selector_gain-${index}`}>Chunk Selector: Gain:</label>
          <input
            type="text"
            id={`chunk_selector_gain-${index}`}
            name="chunk_selector_gain"
            value={camera.chunk_selector_gain || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`chunk_enable_gain-${index}`}>Enable Chunk: Gain:</label>
          <input
            type="checkbox"
            id={`chunk_enable_gain-${index}`}
            name="chunk_enable_gain"
            checked={camera.chunk_enable_gain || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`chunk_selector_timestamp-${index}`}>Chunk Selector: Timestamp:</label>
          <input
            type="text"
            id={`chunk_selector_timestamp-${index}`}
            name="chunk_selector_timestamp"
            value={camera.chunk_selector_timestamp || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`chunk_enable_timestamp-${index}`}>Enable Chunk: Timestamp:</label>
          <input
            type="checkbox"
            id={`chunk_enable_timestamp-${index}`}
            name="chunk_enable_timestamp"
            checked={camera.chunk_enable_timestamp || false}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`adjust_timestamp-${index}`}>Adjust Timestamp:</label>
          <input
            type="checkbox"
            id={`adjust_timestamp-${index}`}
            name="adjust_timestamp"
            checked={camera.adjust_timestamp || false}
            onChange={(e) => handleInputChange(index, e)}
          />
          
          <label htmlFor={`exposure_time-${index}`}>Exposure Time (µs):</label>
          <input
            type="number"
            id={`exposure_time-${index}`}
            name="exposure_time"
            value={camera.exposure_time || 0}
            onChange={(e) => handleInputChange(index, e)}
            disabled={camera.exposure_auto !== 'Off'}
          />

          <label htmlFor={`gain-${index}`}>Gain:</label>
          <input
            type="number"
            id={`gain-${index}`}
            name="gain"
            value={camera.gain || 0}
            onChange={(e) => handleInputChange(index, e)}
            disabled={camera.gain_auto !== 'Off'}
          />

          <label htmlFor={`line2_selector-${index}`}>Line2 Selector:</label>
          <input
            type="text"
            id={`line2_selector-${index}`}
            name="line2_selector"
            value={camera.line2_selector || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`line2_v33enable-${index}`}>Line2 V3.3 Enable:</label>
          <input
            type="checkbox"
            id={`line2_v33enable-${index}`}
            name="line2_v33enable"
            checked={camera.line2_v33enable || false}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`line3_selector-${index}`}>Line3 Selector:</label>
          <input
            type="text"
            id={`line3_selector-${index}`}
            name="line3_selector"
            value={camera.line3_selector || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`line3_linemode-${index}`}>Line3 Line Mode (Input, Output):</label>
          <input
            type="text"
            id={`line3_linemode-${index}`}
            name="line3_linemode"
            value={camera.line3_linemode || ''}
            onChange={(e) => handleInputChange(index, e)}
          />

          <label htmlFor={`trigger_selector-${index}`}>Trigger Selector (FrameStart, FrameEnd):</label>
          <input
            type="text"
            id={`trigger_selector-${index}`}
            name="trigger_selector"
            value={camera.trigger_selector || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`trigger_source-${index}`}>Trigger Source (Line3, Software):</label>
          <input
            type="text"
            id={`trigger_source-${index}`}
            name="trigger_source"
            value={camera.trigger_source || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`trigger_delay-${index}`}>Trigger Delay (µs):</label>
          <input
            type="number"
            id={`trigger_delay-${index}`}
            name="trigger_delay"
            value={camera.trigger_delay || 0}
            onChange={(e) => handleInputChange(index, e)}
          />
          <label htmlFor={`trigger_overlap-${index}`}>Trigger Overlap (ReadOut, Off):</label>
          <input
            type="text"
            id={`trigger_overlap-${index}`}
            name="trigger_overlap"
            value={camera.trigger_overlap || ''}
            onChange={(e) => handleInputChange(index, e)}
          />
          
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
