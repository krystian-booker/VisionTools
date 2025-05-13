const express = require('express');
const expressLayouts = require('express-ejs-layouts');
const path = require('path');
const fs = require('fs').promises;
const yaml = require('js-yaml');
const os = require('os');

const app = express();
const PORT = process.env.PORT || 3000;

// Where your YAML lives
const camerasFile = '/etc/frcVisionTools/cameras.yaml';

// View engine setup
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));
app.use(expressLayouts);
app.use(express.json());
app.set('layout', 'layout');

// Serve static assets
app.use(express.static(path.join(__dirname, 'public')));

// API endpoint to get (or create) the camera config
app.get('/api/cameras', async (req, res) => {
  try {
    // Ensure directory exists
    await fs.mkdir(path.dirname(camerasFile), { recursive: true });

    let fileContents;
    try {
      fileContents = await fs.readFile(camerasFile, 'utf8');
    } catch (err) {
      if (err.code === 'ENOENT') {
        const defaultConfig = { camera_config: { cameras: [] } };
        fileContents = yaml.dump(defaultConfig);
        await fs.writeFile(camerasFile, fileContents, 'utf8');
      } else {
        throw err;
      }
    }

    const data = yaml.load(fileContents);
    res.json(data);
  } catch (err) {
    console.error('Error loading cameras.yaml:', err);
    res.status(500).json({ error: 'Could not load camera config' });
  }
});

// POST /api/cameras - create a new camera entry
app.post('/api/cameras', async (req, res) => {
  const {
    serial,
    name = '',
    role,
    video_mode,
    framerate = 30.0,
    exposure = 9996.41,
    gain = 18.4
  } = req.body;

  if (!serial || !role || !video_mode) {
    return res.status(400).json({ error: 'serial, role and video_mode are required' });
  }

  try {
    await fs.mkdir(path.dirname(camerasFile), { recursive: true });

    let fileContents;
    try {
      fileContents = await fs.readFile(camerasFile, 'utf8');
    } catch (err) {
      if (err.code === 'ENOENT') {
        const defaultConfig = { camera_config: { cameras: [] } };
        fileContents = yaml.dump(defaultConfig, { quotingType: '"', forceQuotes: true });
        await fs.writeFile(camerasFile, fileContents, 'utf8');
      } else {
        throw err;
      }
    }

    const data = yaml.load(fileContents) || {};
    data.camera_config = data.camera_config || {};
    data.camera_config.cameras = data.camera_config.cameras || [];

    const newCamera = {
      serial: String(serial),
      name: String(name),
      role: String(role),
      video_mode: String(video_mode),
      framerate: Number(framerate),
      exposure: Number(exposure),
      gain: Number(gain)
    };

    data.camera_config.cameras.push(newCamera);
    const newYaml = yaml.dump(data, { quotingType: '"', forceQuotes: true });
    await fs.writeFile(camerasFile, newYaml, 'utf8');

    // Return the newly created camera
    res.status(201).json(newCamera);
  } catch (err) {
    console.error('Error saving new camera:', err);
    res.status(500).json({ error: 'Could not save camera configuration' });
  }
});


// PUT /api/cameras/:serial - update an existing camera
app.put('/api/cameras/:serial', async (req, res) => {
  const serial = req.params.serial;
  const {
    name = '',
    role,
    video_mode,
    framerate = 30.0,
    exposure = 9996.41,
    gain = 18.4
  } = req.body;

  if (!role || !video_mode) {
    return res.status(400).json({ error: 'role and video_mode are required' });
  }

  try {
    await fs.mkdir(path.dirname(camerasFile), { recursive: true });
    let contents;
    try {
      contents = await fs.readFile(camerasFile, 'utf8');
    } catch (err) {
      if (err.code === 'ENOENT') {
        return res.status(404).json({ error: 'No cameras file found' });
      }
      throw err;
    }

    const data = yaml.load(contents) || {};
    const cams = data.camera_config?.cameras || [];
    const idx = cams.findIndex(c => c.serial === serial);
    if (idx === -1) {
      return res.status(404).json({ error: 'Camera not found' });
    }

    cams[idx] = {
      ...cams[idx],
      name: String(name),
      role: String(role),
      video_mode: String(video_mode),
      framerate: Number(framerate),
      exposure: Number(exposure),
      gain: Number(gain)
    };
    data.camera_config.cameras = cams;

    // save with forced quoting on strings
    const updatedYaml = yaml.dump(data, { quotingType: '"', forceQuotes: true });
    await fs.writeFile(camerasFile, updatedYaml, 'utf8');

    // return the updated camera
    res.json(cams[idx]);
  } catch (err) {
    console.error('Error updating camera:', err);
    res.status(500).json({ error: 'Could not update camera' });
  }
});


// DELETE /api/cameras/:serial - remove a camera
app.delete('/api/cameras/:serial', async (req, res) => {
  const serial = req.params.serial;
  try {
    await fs.mkdir(path.dirname(camerasFile), { recursive: true });
    let contents;
    try {
      contents = await fs.readFile(camerasFile, 'utf8');
    } catch (err) {
      if (err.code === 'ENOENT') {
        return res.status(404).json({ error: 'No cameras file found' });
      }
      throw err;
    }

    const data = yaml.load(contents) || {};
    const cams = (data.camera_config?.cameras) || [];
    const idx  = cams.findIndex(c => c.serial === serial);
    if (idx === -1) {
      return res.status(404).json({ error: 'Camera not found' });
    }

    cams.splice(idx, 1);
    data.camera_config.cameras = cams;
    await fs.writeFile(camerasFile, yaml.dump(data), 'utf8');

    res.json({ serial });
  } catch (err) {
    console.error('Error deleting camera:', err);
    res.status(500).json({ error: 'Could not delete camera' });
  }
});

// Page routes
app.get('/', (req, res) => {
  const localIP = getLocalIP();
  res.render('dashboard', { active: 'dashboard', localIP });
});

app.get('/camera-setup', (req, res) => {
  res.render('cameraSetup', { active: 'camera-setup' });
});
app.get('/camera-calibration', (req, res) => {
  res.render('cameraCalibration', { active: 'camera-calibration' });
});
app.get('/about', (req, res) => {
  res.render('about', { active: 'about' });
});

function getLocalIP() {
  const interfaces = os.networkInterfaces();
  for (const iface of Object.values(interfaces)) {
    for (const config of iface) {
      if (config.family === 'IPv4' && !config.internal) {
        return config.address;
      }
    }
  }
  return 'localhost';
}


// Start server
app.listen(PORT, () => {
  const interfaces = os.networkInterfaces();
  let localIP = 'localhost';

  // Loop through interfaces to find the first non-internal IPv4 address
  for (const iface of Object.values(interfaces)) {
    for (const config of iface) {
      if (config.family === 'IPv4' && !config.internal) {
        localIP = config.address;
        break;
      }
    }
    if (localIP !== 'localhost') break;
  }

  console.log(`Website running on:`);
  console.log(`http://localhost:${PORT}`);
  console.log(`http://${localIP}:${PORT}`);
});
