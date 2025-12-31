import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthNavbarItem from '../../components/auth/AuthNavbarItem';

export default function NavbarItemWrapper(props) {
  if (props.type === 'custom-auth-navbar-item') {
    return <AuthNavbarItem {...props} />;
  }
  return <NavbarItem {...props} />;
}
