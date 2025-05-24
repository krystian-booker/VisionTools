// frontend/src/components/training/TrainingForm.js
import React, { useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import {
    setFormField,
    startTrainingJob,
    // fetchTrainingConfigOptions is likely called in TrainingScreen or App
} from '../../features/training/trainingSlice';
import {
    Button, TextField, Select, MenuItem, FormControl, InputLabel,
    Grid, Paper, Typography, CircularProgress, Box
} from '@mui/material';

const TrainingForm = () => {
    const dispatch = useDispatch();
    const { formParameters, configOptions, configStatus, jobStatus, error } = useSelector((state) => state.training);

    const isJobActive = jobStatus === 'initiating' || jobStatus === 'running';

    const handleChange = (event) => {
        const { name, value } = event.target;
        dispatch(setFormField({ field: name, value }));
    };

    const handleSubmit = (event) => {
        event.preventDefault();
        dispatch(startTrainingJob(formParameters));
    };

    if (configStatus === 'loading') {
        return (
            <Paper elevation={3} sx={{ p: 3, mt: 2, textAlign: 'center' }}>
                <CircularProgress />
                <Typography sx={{ mt: 1 }}>Loading configuration...</Typography>
            </Paper>
        );
    }

    if (configStatus === 'failed') {
        return (
            <Paper elevation={3} sx={{ p: 3, mt: 2, textAlign: 'center' }}>
                <Typography color="error">Failed to load training configuration. Please try refreshing.</Typography>
            </Paper>
        );
    }
    
    return (
        <Paper elevation={3} sx={{ p: 3, mt: 2 }}>
            <Typography variant="h6" gutterBottom>
                Training Configuration
            </Typography>
            <form onSubmit={handleSubmit}>
                <Grid container spacing={2}>
                    <Grid item xs={12}>
                        <TextField
                            fullWidth
                            label="Path to data.yaml"
                            name="data_yaml_path"
                            value={formParameters.data_yaml_path}
                            onChange={handleChange}
                            required
                            disabled={isJobActive}
                        />
                    </Grid>
                    <Grid item xs={12} sm={6}>
                        <FormControl fullWidth required disabled={isJobActive}>
                            <InputLabel id="model-type-label">Model Type</InputLabel>
                            <Select
                                labelId="model-type-label"
                                name="model_type"
                                value={formParameters.model_type}
                                label="Model Type"
                                onChange={handleChange}
                            >
                                {configOptions.model_types && configOptions.model_types.map((type) => (
                                    <MenuItem key={type} value={type}>{type}</MenuItem>
                                ))}
                            </Select>
                        </FormControl>
                    </Grid>
                    <Grid item xs={12} sm={6}>
                        <FormControl fullWidth disabled={isJobActive}>
                            <InputLabel id="conversion-target-label">Conversion Target (Optional)</InputLabel>
                            <Select
                                labelId="conversion-target-label"
                                name="conversion_target"
                                value={formParameters.conversion_target || ''} // Ensure empty string for "None"
                                label="Conversion Target (Optional)"
                                onChange={handleChange}
                            >
                                <MenuItem value=""><em>None (Train Only)</em></MenuItem>
                                {configOptions.conversion_targets && configOptions.conversion_targets.map((target) => (
                                    <MenuItem key={target} value={target}>{target}</MenuItem>
                                ))}
                            </Select>
                        </FormControl>
                    </Grid>
                    <Grid item xs={12} sm={4}>
                        <TextField
                            fullWidth
                            label="Epochs"
                            name="epochs"
                            type="number"
                            value={formParameters.epochs}
                            onChange={handleChange}
                            required
                            disabled={isJobActive}
                            InputProps={{ inputProps: { min: 1 } }}
                        />
                    </Grid>
                    <Grid item xs={12} sm={4}>
                        <TextField
                            fullWidth
                            label="Image Size (imgsz)"
                            name="imgsz"
                            type="number"
                            value={formParameters.imgsz}
                            onChange={handleChange}
                            required
                            disabled={isJobActive}
                            InputProps={{ inputProps: { min: 32, step: 32 } }} // Example constraints
                        />
                    </Grid>
                    <Grid item xs={12} sm={4}>
                        <FormControl fullWidth required disabled={isJobActive}>
                            <InputLabel id="device-label">Device</InputLabel>
                            <Select
                                labelId="device-label"
                                name="device"
                                value={formParameters.device}
                                label="Device"
                                onChange={handleChange}
                            >
                                <MenuItem value="cpu">CPU</MenuItem>
                                {/* Dynamic GPU options could be added if discoverable */}
                                <MenuItem value="0">GPU 0</MenuItem> 
                            </Select>
                        </FormControl>
                    </Grid>
                    <Grid item xs={12}>
                        <TextField
                            fullWidth
                            label="Run Name (Optional)"
                            name="run_name"
                            value={formParameters.run_name}
                            onChange={handleChange}
                            disabled={isJobActive}
                            placeholder="e.g., my_experiment_run"
                        />
                    </Grid>
                </Grid>
                <Box sx={{ mt: 3, display: 'flex', alignItems: 'center' }}>
                    <Button
                        type="submit"
                        variant="contained"
                        color="primary"
                        disabled={isJobActive || configStatus !== 'succeeded'}
                        sx={{ mr: 1 }}
                    >
                        {isJobActive ? 'Processing...' : 'Start Training'}
                    </Button>
                    {isJobActive && <CircularProgress size={24} />}
                </Box>
                {jobStatus === 'failed' && error && (
                     <Typography color="error" sx={{mt:1}}>Last attempt failed: {typeof error === 'object' ? JSON.stringify(error) : error}</Typography>
                )}
            </form>
        </Paper>
    );
};

export default TrainingForm;
