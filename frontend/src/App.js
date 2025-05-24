// frontend/src/App.js
import React from 'react';
import RosNodeList from './components/ros/RosNodeList'; // For ROS Node Management
import TrainingScreen from './screens/TrainingScreen';   // For YOLO Training UI

import { ThemeProvider, createTheme } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import Container from '@mui/material/Container';
import Typography from '@mui/material/Typography';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Box from '@mui/material/Box';
import Divider from '@mui/material/Divider';

// A basic theme for MUI (can be customized)
const theme = createTheme({
    palette: {
        mode: 'light', // Or 'dark'
        primary: {
            main: '#1976d2', 
        },
        secondary: {
            main: '#dc004e', 
        },
    },
    typography: {
        h1: {
          fontSize: '2.2rem', // Slightly smaller for section titles
          fontWeight: 500,
          marginTop: '20px',
          marginBottom: '10px',
        },
        h4: { // Used as main screen titles
            fontSize: '1.8rem',
            fontWeight: 500,
            marginTop: '10px',
            marginBottom: '20px',
        }
    }
});

function App() {
    return (
        <ThemeProvider theme={theme}>
            <CssBaseline /> 
            <AppBar position="static">
                <Toolbar>
                    <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
                        Robotics & ML Control Dashboard
                    </Typography>
                </Toolbar>
            </AppBar>
            <Container maxWidth="xl"> {/* Using "xl" for wider content area */}
                <Box sx={{ my: 2 }}> {/* my: margin top and bottom */}
                    
                    {/* Section 1: YOLO Model Training & Conversion */}
                    {/* TrainingScreen includes its own Typography for title */}
                    <TrainingScreen />

                    <Divider sx={{ my: 4 }} /> {/* Visual separator */}

                    {/* Section 2: ROS Node Status & Management */}
                    <Typography variant="h4" component="h1" gutterBottom>
                        ROS Node Status & Management
                    </Typography>
                    <RosNodeList />
                </Box>
            </Container>
        </ThemeProvider>
    );
}

export default App;
