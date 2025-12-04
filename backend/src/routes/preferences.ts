import { Router, Response } from 'express';
import { getUserPreferences, updateUserPreferences } from '../lib/user-preferences';

/**
 * User preferences routes
 */

const router = Router();

/**
 * GET /api/preferences
 * Get user preferences
 */
router.get('/', async (req: any, res: Response) => {
    try {
        const userId = req.userId;
        const preferences = await getUserPreferences(userId);

        res.json({
            success: true,
            preferences
        });

    } catch (error) {
        console.error('Get preferences error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to fetch preferences'
        });
    }
});

/**
 * PUT /api/preferences
 * Update user preferences
 */
router.put('/', async (req: any, res: Response) => {
    try {
        const userId = req.userId;
        const { theme, language, chatSettings } = req.body;

        const updated = await updateUserPreferences(userId, {
            theme,
            language,
            chatSettings
        });

        res.json({
            success: true,
            preferences: updated
        });

    } catch (error) {
        console.error('Update preferences error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to update preferences'
        });
    }
});

export default router;
