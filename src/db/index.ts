import { drizzle } from 'drizzle-orm/neon-http';
import { neon } from '@neondatabase/serverless';

// Initialize Neon connection
const sql = neon(process.env.DATABASE_URL!);

// Export Drizzle database instance
export const db = drizzle(sql);
