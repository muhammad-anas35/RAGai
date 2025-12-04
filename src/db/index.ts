import { drizzle } from 'drizzle-orm/neon-http';
import { neon } from '@neondatabase/serverless';
import dotenv from 'dotenv';
import path from 'path';
import { fileURLToPath } from 'url';

// Load environment variables if not already loaded
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const envPath = path.resolve(__dirname, '../../.env.local');
dotenv.config({ path: envPath });

// Verify DATABASE_URL is set
if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL environment variable is not set. Please create .env.local with DATABASE_URL from Neon.');
}

// Initialize Neon connection
const sql = neon(process.env.DATABASE_URL);

// Export Drizzle database instance
export const db = drizzle(sql);
