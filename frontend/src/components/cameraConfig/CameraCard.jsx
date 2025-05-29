import React from 'react';
import {
  TextField, Checkbox, Select, MenuItem, FormControl, InputLabel,
  Typography, Paper, Grid, Box, IconButton, FormControlLabel
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';

function CameraCard({ camera, index, handleInputChange, handleRemoveCamera }) {
  return (
    <Paper elevation={3} sx={{ p: 2, mb: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
        <Typography variant="h6" component="h2" gutterBottom>
          {camera.name || `Camera ${index + 1}`}
        </Typography>
        <IconButton aria-label="delete camera" onClick={() => handleRemoveCamera(index)} color="error">
          <DeleteIcon />
        </IconButton>
      </Box>
      <Grid container spacing={2}>
        {/* General Settings */}
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Camera Name" name="name" value={camera.name || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Serial (Mandatory)" name="serial" value={camera.serial || ''} onChange={(e) => handleInputChange(index, e)} fullWidth required margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Topic" name="topic" value={camera.topic || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>

        {/* Boolean Toggles */}
        <Grid item xs={12} sm={6} md={4}>
          <FormControlLabel control={<Checkbox name="debug" checked={camera.debug || false} onChange={(e) => handleInputChange(index, e)} />} label="Debug Mode" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <FormControlLabel control={<Checkbox name="compute_brightness" checked={camera.compute_brightness || false} onChange={(e) => handleInputChange(index, e)} />} label="Compute Brightness" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <FormControlLabel control={<Checkbox name="dump_node_map" checked={camera.dump_node_map || false} onChange={(e) => handleInputChange(index, e)} />} label="Dump Node Map" />
        </Grid>
        
        {/* Auto Settings */}
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Gain Auto" name="gain_auto" value={camera.gain_auto || 'Off'} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="Off, Once, Continuous" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Exposure Auto" name="exposure_auto" value={camera.exposure_auto || 'Off'} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="Off, Once, Continuous" />
        </Grid>

        {/* Image Settings */}
        <Grid item xs={12} sm={6} md={3}>
          <TextField label="Offset X" name="offset_x" type="number" value={camera.offset_x || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <TextField label="Offset Y" name="offset_y" type="number" value={camera.offset_y || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <TextField label="Image Width" name="image_width" type="number" value={camera.image_width || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <TextField label="Image Height" name="image_height" type="number" value={camera.image_height || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <FormControl fullWidth margin="dense">
            <InputLabel id={`pixel_format-label-${index}`}>Pixel Format</InputLabel>
            <Select
              labelId={`pixel_format-label-${index}`}
              name="pixel_format"
              value={camera.pixel_format || 'Mono8'}
              onChange={(e) => handleInputChange(index, e)}
              label="Pixel Format"
            >
              <MenuItem value="Mono8">Mono8</MenuItem>
              <MenuItem value="BayerRG8">BayerRG8</MenuItem>
              <MenuItem value="RGB8">RGB8</MenuItem>
            </Select>
          </FormControl>
        </Grid>

        {/* Frame Rate */}
          <Grid item xs={12} sm={6} md={4}>
          <FormControlLabel control={<Checkbox name="frame_rate_continuous" checked={camera.frame_rate_continuous || false} onChange={(e) => handleInputChange(index, e)} />} label="Frame Rate Continuous" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Frame Rate (fps)" name="frame_rate" type="number" inputProps={{step: "0.1"}} value={camera.frame_rate || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        
        {/* Trigger Settings */}
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Trigger Mode" name="trigger_mode" value={camera.trigger_mode || 'Off'} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="On, Off" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Trigger Selector" name="trigger_selector" value={camera.trigger_selector || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="FrameStart, FrameEnd" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Trigger Source" name="trigger_source" value={camera.trigger_source || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="Line3, Software" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Trigger Delay (µs)" name="trigger_delay" type="number" value={camera.trigger_delay || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Trigger Overlap" name="trigger_overlap" value={camera.trigger_overlap || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="ReadOut, Off" />
        </Grid>

        {/* Chunk Data */}
        <Grid item xs={12}><Typography variant="subtitle1" sx={{mt:1}}>Chunk Data</Typography></Grid>
        <Grid item xs={12} sm={6} md={3}>
          <FormControlLabel control={<Checkbox name="chunk_mode_active" checked={camera.chunk_mode_active || false} onChange={(e) => handleInputChange(index, e)} />} label="Chunk Mode Active" />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <FormControlLabel control={<Checkbox name="adjust_timestamp" checked={camera.adjust_timestamp || false} onChange={(e) => handleInputChange(index, e)} />} label="Adjust Timestamp" />
        </Grid>
        
        <Grid item xs={12} sm={6} md={4}> <TextField label="Chunk Selector: Frame ID" name="chunk_selector_frame_id" value={camera.chunk_selector_frame_id || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={2}> <FormControlLabel control={<Checkbox name="chunk_enable_frame_id" checked={camera.chunk_enable_frame_id || false} onChange={(e) => handleInputChange(index, e)} />} label="Enable" /> </Grid>
        
        <Grid item xs={12} sm={6} md={4}> <TextField label="Chunk Selector: Exposure Time" name="chunk_selector_exposure_time" value={camera.chunk_selector_exposure_time || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={2}> <FormControlLabel control={<Checkbox name="chunk_enable_exposure_time" checked={camera.chunk_enable_exposure_time || false} onChange={(e) => handleInputChange(index, e)} />} label="Enable" /> </Grid>

        <Grid item xs={12} sm={6} md={4}> <TextField label="Chunk Selector: Gain" name="chunk_selector_gain" value={camera.chunk_selector_gain || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={2}> <FormControlLabel control={<Checkbox name="chunk_enable_gain" checked={camera.chunk_enable_gain || false} onChange={(e) => handleInputChange(index, e)} />} label="Enable" /> </Grid>

        <Grid item xs={12} sm={6} md={4}> <TextField label="Chunk Selector: Timestamp" name="chunk_selector_timestamp" value={camera.chunk_selector_timestamp || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={2}> <FormControlLabel control={<Checkbox name="chunk_enable_timestamp" checked={camera.chunk_enable_timestamp || false} onChange={(e) => handleInputChange(index, e)} />} label="Enable" /> </Grid>

        {/* Manual Exposure/Gain */}
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Exposure Time (µs)" name="exposure_time" type="number" value={camera.exposure_time || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" disabled={camera.exposure_auto !== 'Off'} />
        </Grid>
        <Grid item xs={12} sm={6} md={4}>
          <TextField label="Gain" name="gain" type="number" value={camera.gain || 0} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" disabled={camera.gain_auto !== 'Off'} />
        </Grid>

        {/* Line Settings */}
        <Grid item xs={12}><Typography variant="subtitle1" sx={{mt:1}}>Line Settings</Typography></Grid>
        <Grid item xs={12} sm={6} md={3}> <TextField label="Line2 Selector" name="line2_selector" value={camera.line2_selector || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={3}> <FormControlLabel control={<Checkbox name="line2_v33enable" checked={camera.line2_v33enable || false} onChange={(e) => handleInputChange(index, e)} />} label="Line2 V3.3 Enable" /> </Grid>
        
        <Grid item xs={12} sm={6} md={3}> <TextField label="Line3 Selector" name="line3_selector" value={camera.line3_selector || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" /> </Grid>
        <Grid item xs={12} sm={6} md={3}> <TextField label="Line3 Line Mode" name="line3_linemode" value={camera.line3_linemode || ''} onChange={(e) => handleInputChange(index, e)} fullWidth margin="dense" helperText="Input, Output" /> </Grid>
      </Grid>
    </Paper>
  );
}

export default CameraCard;
