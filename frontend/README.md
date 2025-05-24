# Frontend Web Application (React)

## Purpose

This directory contains the frontend single-page application (SPA) built with React. It provides a modern, interactive user interface for managing the YOLOv5 training pipeline and controlling ROS 2 nodes via the backend server.

## Technology Stack

-   **React**: A JavaScript library for building user interfaces.
-   **Redux (Redux Toolkit)**: For predictable state management across the application.
    -   `@reduxjs/toolkit`: The recommended way to write Redux logic.
    -   `react-redux`: For connecting React components to the Redux store.
-   **Material UI (MUI)**: A popular React UI framework for pre-built components and styling, enabling a consistent and professional look and feel.
    -   `@mui/material`: Core MUI components.
    -   `@mui/icons-material`: For Material Design icons.
-   **Axios**: A promise-based HTTP client for making API requests to the backend. (Alternatively, the native Fetch API could be used).
-   **JavaScript (ES6+)**: The primary programming language.
-   **CSS**: Styling is primarily handled via Material UI's styling solutions (e.g., `sx` prop, `styled-components` if used).

## Project Structure (`frontend/src/`)

-   **`index.js`**: The main entry point that renders the React application into the DOM (typically into `<div id="root"></div>` in `public/index.html`). It also wraps the application with the Redux `<Provider>`.
-   **`App.js`**: The root React component that sets up the main layout, routing (if applicable), and global UI elements like the `AppBar` and `ThemeProvider` for Material UI.
-   **`store.js`**: Configures the Redux store, combining all reducers (slices) from the application.
-   **`components/`**: Contains reusable UI components.
    -   `ros/`: Components specific to ROS node management (e.g., `RosNodeList.js`).
    -   `training/`: Components specific to the YOLO training UI (e.g., `TrainingForm.js`, `LogDisplay.js`, `JobStatusDisplay.js`).
-   **`features/`**: Contains Redux "slices" using Redux Toolkit's `createSlice` and `createAsyncThunk`. Each feature typically manages its own state.
    -   `rosNodes/rosNodesSlice.js`: Manages state related to ROS nodes (list, statuses, operations).
    -   `training/trainingSlice.js`: Manages state related to the YOLO training pipeline (form parameters, job status, logs, configuration).
-   **`screens/`**: Higher-level components that represent different "pages" or sections of the application, often composing multiple smaller components.
    -   `TrainingScreen.js`: The main screen for the YOLO training and conversion UI.
    -   *(A `RosNodesScreen.js` could be created to encapsulate `RosNodeList` if more layout around it is needed).*

## Key Features

### 1. YOLO Model Training & Conversion UI (`TrainingScreen.js`)

-   **Dynamic Configuration**: Fetches training parameters (model types, conversion targets, defaults) from the backend on load.
-   **Parameter Input**: Allows users to configure dataset path, model type, epochs, image size, device, conversion target, and run name.
-   **Job Initiation**: Starts the training and conversion job on the backend.
-   **Real-time Log Streaming**: Displays logs from the backend training process in real time using Server-Sent Events (SSE).
-   **Status Monitoring**:
    -   Shows the current status of the training job (e.g., 'initiating', 'running', 'completed', 'failed').
    -   Periodically polls the backend for status updates while a job is active.
    -   Displays paths to the generated best model (`.pt`) and converted model files upon successful completion.
    -   Shows error messages if the job fails.
-   **State Management**: Uses the `trainingSlice` in Redux to manage form state, job status, logs, and API interaction.

### 2. ROS Node Management UI (`RosNodeList.js` within `App.js`)

-   **Node Listing**: Fetches and displays a list of ROS 2 nodes from the backend database, showing details like name, description, package, type, current status, PID, and last start/stop times.
-   **Enable/Disable**: Allows users to toggle the `is_enabled` status of each node using a Switch component.
-   **Lifecycle Control**: Provides buttons to Start, Stop, and Restart individual ROS nodes.
    -   Button states (enabled/disabled) are managed based on the node's current `status` and `is_enabled` flag to prevent invalid operations.
-   **Real-time Feedback**: The UI updates to reflect changes in node status (e.g., 'running', 'stopped', 'starting', 'stopping') as operations are performed.
-   **State Management**: Uses the `rosNodesSlice` in Redux to manage the list of nodes, their individual states, and API interactions for control operations.

## API Interaction

-   All communication with the backend Flask server (e.g., `http://localhost:5000`) is handled via asynchronous HTTP requests made using `axios`.
-   Redux Toolkit's `createAsyncThunk` is used to manage these API calls, handling pending, fulfilled, and rejected states automatically, and updating the Redux store accordingly.
-   Server-Sent Events (SSE) are used for the training log stream to provide real-time updates without constant polling for logs.

## Setup & Running

1.  **Navigate to the `frontend/` directory**:
    ```bash
    cd frontend
    ```
2.  **Install Dependencies**:
    If not already done (e.g., by the root `README.md` instructions), install all Node.js dependencies:
    ```bash
    npm install
    ```
    Key dependencies include:
    -   `react`, `react-dom`
    -   `@reduxjs/toolkit`, `react-redux`
    -   `@mui/material`, `@emotion/react`, `@emotion/styled`, `@mui/icons-material`
    -   `axios`

3.  **Run the Development Server**:
    The `start_all.sh` script in the project root is the recommended way to start both the backend and frontend. It typically runs this command for you.
    To run the frontend development server manually:
    ```bash
    npm start
    ```
    This will usually open the application in your default web browser at `http://localhost:3000`. The page will auto-reload if you make changes to the source code.

## Building for Production (Optional)

If you need to create a production build of the frontend:
```bash
npm run build
```
This command bundles the app into static files in the `build/` directory. These files are optimized for performance and can be deployed to any static file hosting service.
