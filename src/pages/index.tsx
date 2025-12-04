import type { ReactNode } from 'react';
import { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';

export default function Home(): ReactNode {
  const history = useHistory();
  
  useEffect(() => {
    // Redirect to login page
    history.push('/auth/login');
  }, [history]);

  return null;
}
