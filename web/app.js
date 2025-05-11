const express = require('express');
const expressLayouts = require('express-ejs-layouts');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;

// View engine setup
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));
app.use(expressLayouts);
app.set('layout', 'layout');

// Serve static assets
app.use(express.static(path.join(__dirname, 'public')));

// Routes
app.get('/', (req, res) => {
  res.render('dashboard', { active: 'dashboard' });
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

// Start server
app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});