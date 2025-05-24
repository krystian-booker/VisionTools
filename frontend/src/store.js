// frontend/src/store.js
import { configureStore } from '@reduxjs/toolkit';
import rosNodesReducer from './features/rosNodes/rosNodesSlice';
import trainingReducer from './features/training/trainingSlice'; // Import the new training reducer

export const store = configureStore({
    reducer: {
        rosNodes: rosNodesReducer,
        training: trainingReducer, // Add the training reducer
        // other reducers can be added here
    },
    // It's good practice to disable serializableCheck for Redux Toolkit
    // if you plan to store non-serializable values like EventSource instances or complex objects,
    // though for this specific setup, we are mostly storing serializable data in Redux state.
    // However, some action payloads from errors might be non-serializable.
    middleware: (getDefaultMiddleware) =>
        getDefaultMiddleware({
            serializableCheck: {
                // Ignore these action types, or specific paths in actions/state
                // ignoredActions: ['training/setSseInstance'], // Example if you were to store EventSource
                ignoredActionPaths: ['meta.arg', 'payload.config', 'payload.headers', 'payload.request', 'error'], // Common non-serializable paths in axios errors
                ignoredPaths: ['some.path.to.nonSerializableValue'], // Example for state path
            },
        }),
});
