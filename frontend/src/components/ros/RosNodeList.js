// frontend/src/components/ros/RosNodeList.js
import React, { useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { 
    fetchRosNodes, 
    toggleNodeEnabled,
    startRosNode, 
    stopRosNode, 
    restartRosNode 
} from '../../features/rosNodes/rosNodesSlice'; 

import {
    Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Paper,
    CircularProgress, Typography, Box, Switch, Button, ButtonGroup, Chip,
    Tooltip, IconButton 
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import StopIcon from '@mui/icons-material/Stop';
import RefreshIcon from '@mui/icons-material/Refresh';
// PowerSettingsNewIcon could be used for enable/disable if preferred over Switch in actions column

// Helper to determine if a node is in a transitional state
const isNodeTransitioning = (nodeStatus) => {
    return ['starting', 'stopping', 'restarting', 'toggling'].includes(nodeStatus);
};


const RosNodeList = () => {
    const dispatch = useDispatch();
    const { nodes, status: globalListStatus, error } = useSelector((state) => state.rosNodes);

    useEffect(() => {
        // Fetch nodes when component mounts or if global status is idle (e.g., after an error)
        if (globalListStatus === 'idle' || globalListStatus === 'failed') { 
            dispatch(fetchRosNodes());
        }
        // No need to re-fetch periodically here; user actions will update individual nodes
        // and critical errors might trigger a global 'failed' state, prompting re-fetch.
    }, [globalListStatus, dispatch]);

    const handleToggleEnabled = (nodeId, currentIsEnabled) => {
        dispatch(toggleNodeEnabled({ nodeId, isEnabled: !currentIsEnabled }));
    };

    const handleStartNode = (nodeId) => {
        dispatch(startRosNode(nodeId));
    };

    const handleStopNode = (nodeId) => {
        dispatch(stopRosNode(nodeId));
    };
    
    const handleRestartNode = (nodeId) => {
        dispatch(restartRosNode(nodeId));
    };


    if (globalListStatus === 'loading' && nodes.length === 0) { // Show global loading only if no nodes are present yet
        return <Box display="flex" justifyContent="center" alignItems="center" minHeight="200px"><CircularProgress /></Box>;
    }

    // Show global error if fetching list failed, and no nodes are available to display
    if (globalListStatus === 'failed' && nodes.length === 0) {
        return <Typography color="error" align="center">Error fetching ROS nodes: {error || 'Unknown error'}</Typography>;
    }
    
    // If fetch succeeded but list is empty
    if (globalListStatus === 'succeeded' && nodes.length === 0) {
        return (
            <Box sx={{textAlign: 'center', mt: 4}}>
                 <Typography>No ROS nodes found in the database.</Typography>
            </Box>
        );
    }


    return (
        <>
            {/* Show global loading indicator at top if list is already populated but a fetch is in progress */}
            {globalListStatus === 'loading' && nodes.length > 0 && (
                 <Box sx={{ display: 'flex', justifyContent: 'center', my: 2 }}>
                    <CircularProgress size={24} /> <Typography sx={{ ml: 1 }}>Refreshing node list...</Typography>
                 </Box>
            )}
            {/* Show global error if a refresh fails, but keep showing stale data */}
            {globalListStatus === 'failed' && nodes.length > 0 && (
                 <Typography color="error" align="center" sx={{mb:1}}>Error refreshing ROS nodes: {error || 'Unknown error'}. Displaying last known data.</Typography>
            )}

            <TableContainer component={Paper} sx={{ mt: 2 }}>
                <Table sx={{ minWidth: 650 }} aria-label="ros nodes table">
                    <TableHead>
                        <TableRow sx={{ '& th': { fontWeight: 'bold' } }}>
                            <TableCell>Name (Package)</TableCell>
                            <TableCell>Description</TableCell>
                            <TableCell align="center">Enabled</TableCell>
                            <TableCell align="center">Status</TableCell>
                            <TableCell align="center">PID</TableCell>
                            <TableCell>Last Started</TableCell>
                            <TableCell>Last Stopped</TableCell>
                            <TableCell align="center">Actions</TableCell>
                        </TableRow>
                    </TableHead>
                    <TableBody>
                        {nodes.map((node) => {
                            const nodeIsBusy = isNodeTransitioning(node.status);
                            return (
                                <TableRow 
                                    key={node.id}
                                    sx={{ 
                                        '&:last-child td, &:last-child th': { border: 0 },
                                        opacity: nodeIsBusy ? 0.6 : 1, // Visual feedback for busy node
                                        pointerEvents: nodeIsBusy ? 'none' : 'auto' // Disable interactions on busy row
                                    }}
                                >
                                    <TableCell component="th" scope="row">
                                        <Typography variant="subtitle2">{node.node_name}</Typography>
                                        <Typography variant="caption" display="block" color="textSecondary">
                                            {node.package_name}
                                        </Typography>
                                        <Typography variant="caption" display="block" color="textSecondary" sx={{fontSize: '0.7rem'}}>
                                            ({node.node_type}: {node.launch_file_or_executable})
                                        </Typography>
                                    </TableCell>
                                    <TableCell>{node.description}</TableCell>
                                    <TableCell align="center">
                                        <Tooltip title={node.is_enabled ? "Node is Enabled" : "Node is Disabled"}>
                                            <Switch
                                                checked={Boolean(node.is_enabled)}
                                                onChange={() => handleToggleEnabled(node.id, Boolean(node.is_enabled))}
                                                disabled={nodeIsBusy} 
                                                color="primary"
                                            />
                                        </Tooltip>
                                    </TableCell>
                                    <TableCell align="center">
                                        <Chip 
                                            label={node.status || 'unknown'}
                                            color={
                                                node.status === 'running' ? 'success' :
                                                node.status === 'stopped' ? 'default' :
                                                node.status === 'error' ? 'error' :
                                                node.status === 'starting' || node.status === 'stopping' || node.status === 'restarting' || node.status === 'toggling' ? 'info' :
                                                'warning' 
                                            }
                                            size="small"
                                            sx={{minWidth: '70px'}}
                                        />
                                        {nodeIsBusy && <CircularProgress size={12} sx={{ml:1}}/>}
                                    </TableCell>
                                    <TableCell align="center">{node.pid || '-'}</TableCell>
                                    <TableCell>{node.last_started ? new Date(node.last_started).toLocaleString() : '-'}</TableCell>
                                    <TableCell>{node.last_stopped ? new Date(node.last_stopped).toLocaleString() : '-'}</TableCell>
                                    <TableCell align="center">
                                        <ButtonGroup variant="outlined" size="small" aria-label="node actions button group">
                                            <Tooltip title="Start Node">
                                                <span> {/* Span needed for Tooltip when button is disabled */}
                                                    <IconButton
                                                        onClick={() => handleStartNode(node.id)} 
                                                        disabled={!node.is_enabled || node.status === 'running' || nodeIsBusy}
                                                        color="success"
                                                        size="small"
                                                    >
                                                        <PlayArrowIcon />
                                                    </IconButton>
                                                </span>
                                            </Tooltip>
                                            <Tooltip title="Stop Node">
                                                <span>
                                                    <IconButton
                                                        onClick={() => handleStopNode(node.id)} 
                                                        disabled={!(node.status === 'running' || node.status === 'starting' || node.status === 'restarting')} // Allow stopping if in a start-like transition
                                                        color="error"
                                                        size="small"
                                                    >
                                                        <StopIcon />
                                                    </IconButton>
                                                </span>
                                            </Tooltip>
                                            <Tooltip title="Restart Node">
                                                <span>
                                                    <IconButton
                                                        onClick={() => handleRestartNode(node.id)}
                                                        disabled={!node.is_enabled || nodeIsBusy && node.status !== 'running' && node.status !== 'error'} // More nuanced: allow restart if enabled and not busy with another op, or if it's running/error
                                                        size="small"
                                                    >
                                                        <RefreshIcon />
                                                    </IconButton>
                                                </span>
                                            </Tooltip>
                                        </ButtonGroup>
                                    </TableCell>
                                </TableRow>
                            );
                        })}
                    </TableBody>
                </Table>
            </TableContainer>
        </>
    );
};

export default RosNodeList;
